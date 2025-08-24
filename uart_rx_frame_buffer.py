#!/usr/bin/env python3
# Tang Nano 20K — UART -> SDRAM framebuffer (double-buffered) -> HDMI 640x480@60
#
# ESP32-S3 protocol:
#   SOF:  0xAA 0x44
#   ROW:  0xAA 0x55 'R' y_lo y_hi + 1280 bytes (RGB565 LE, 640 pixels)
#         <- FPGA replies 'K' AFTER writing the row to SDRAM
#   EOF:  0xAA 0x66
#         <- FPGA replies 'F' AFTER swapping back/front SDRAM frames at a frame boundary
#
# At boot, both SDRAM frames are cleared to BLACK. HDMI stays black until the
# front clear completes. Line prefetch now starts at DE rising (start-of-line)
# to avoid blocky artifacts.

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import LiteXModule, ClockDomainsRenamer
from litex.build.io import DDROutput
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.uart  import UARTPHY
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.interconnect import wishbone, stream
from litex.soc.interconnect.stream import SyncFIFO

from litedram.phy import GENSDRPHY
from litedram.modules import M12L64322A

# Prefer local platform module if present
try:
    from platforms import sipeed_tang_nano_20k
except Exception:
    from litex_boards.platforms import sipeed_tang_nano_20k


# ---------------- Video timings: 640x480@60 ----------------
H_VISIBLE   = 640
V_VISIBLE   = 480
H_FP        = 16
H_SYNC      = 96
H_BP        = 48
H_TOTAL     = H_VISIBLE + H_FP + H_SYNC + H_BP

V_FP        = 10
V_SYNC      = 2
V_BP        = 33
V_TOTAL     = V_VISIBLE + V_FP + V_SYNC + V_BP

PIXCLK      = 25_200_000
PIX5X       = PIXCLK * 5

# ---------------- Clocks / UART ----------------
SYS_CLK_FREQ = int(48e6)    # stable + UART-friendly
UART_BAUD    = 6_000_000    # fast; backpressure-safe


# -------- SDRAM pads wrapper (discrete pin names seen on TN20K designs) -----------
class _SDRAMPads:
    """
    Discrete SDRAM pins:
      O_sdram_clk, O_sdram_cke, O_sdram_cs_n, O_sdram_cas_n,
      O_sdram_ras_n, O_sdram_wen_n, O_sdram_dqm, O_sdram_addr,
      O_sdram_ba, IO_sdram_dq
    """
    def __init__(self, platform):
        self.clk   = platform.request("O_sdram_clk")
        self.cke   = platform.request("O_sdram_cke")
        self.cs_n  = platform.request("O_sdram_cs_n")
        self.cas_n = platform.request("O_sdram_cas_n")
        self.ras_n = platform.request("O_sdram_ras_n")
        self.we_n  = platform.request("O_sdram_wen_n")   # LiteDRAM expects we_n
        self.dm    = platform.request("O_sdram_dqm")     # width 2
        self.a     = platform.request("O_sdram_addr")    # width 13
        self.ba    = platform.request("O_sdram_ba")      # width 2
        self.dq    = platform.request("IO_sdram_dq")     # width 16 (true inout)


# ---------------- CRG ----------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=SYS_CLK_FREQ):
        self.cd_sys    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")

        # System PLL
        self.submodules.syspll = syspll = GW2APLL(devicename=platform.devicename, device=platform.device)
        syspll.register_clkin(clk27, 27e6)
        syspll.create_clkout(self.cd_sys, sys_clk_freq)

        # HDMI PLL (5x pixel) + /5 divider to get 1x pixel
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIX5X, margin=1e-2)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5", i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk, o_CLKOUT=self.cd_hdmi.clk
        )


# ---------------- Simple HDMI timing gen ----------------
class SimpleTimingHDMI(LiteXModule):
    def __init__(self):
        self.de     = Signal()
        self.hsync  = Signal()
        self.vsync  = Signal()
        self.hcount = Signal(16)
        self.vcount = Signal(16)

        HS_BEG = H_VISIBLE + H_FP
        HS_END = HS_BEG + H_SYNC
        VS_BEG = V_VISIBLE + V_FP
        VS_END = VS_BEG + V_SYNC

        self.sync.hdmi += [
            If(self.hcount == (H_TOTAL - 1),
                self.hcount.eq(0),
                If(self.vcount == (V_TOTAL - 1),
                    self.vcount.eq(0)
                ).Else(
                    self.vcount.eq(self.vcount + 1)
                )
            ).Else(
                self.hcount.eq(self.hcount + 1)
            ),
            self.de.eq((self.hcount < H_VISIBLE) & (self.vcount < V_VISIBLE)),
            self.hsync.eq(~((self.hcount >= HS_BEG) & (self.hcount < HS_END))),  # active-low
            self.vsync.eq(~((self.vcount >= VS_BEG) & (self.vcount < VS_END))),  # active-low
        ]


# ---------------- Helpers ----------------
def rgb565_to_888(pix16):
    r5 = pix16[11:16]; g6 = pix16[5:11]; b5 = pix16[0:5]
    r = Cat(r5, r5[2:5])
    g = Cat(g6, g6[4:6])
    b = Cat(b5, b5[2:5])
    return r, g, b


# ---------------- UART protocol receiver (sys) ----------------
class ProtoUARTRx(LiteXModule):
    """
    Eats:
      SOF: AA 44
      ROW: AA 55 'R' y_lo y_hi + 1280 bytes (RGB565 LE)
      EOF: AA 66
    Outputs (sys):
      - row_fifo_sink: stream of 16-bit pixels (640 words)
      - row_y: row index
      - row_done: 1-cycle pulse after 640 words accepted
      - sof_pulse / eof_pulse
    """
    def __init__(self, uart_source, row_fifo_sink):
        rx  = uart_source
        dst = row_fifo_sink

        self.row_y     = Signal(16)
        self.row_done  = Signal()
        self.sof_pulse = Signal()
        self.eof_pulse = Signal()

        words_out = Signal(10)  # 0..639
        low8      = Signal(8)
        have_lo   = Signal()

        self.comb += [dst.valid.eq(0), dst.data.eq(0), rx.ready.eq(0)]

        fsm = FSM(reset_state="WAIT_A"); self.submodules += fsm

        fsm.act("WAIT_A",
            rx.ready.eq(1),
            If(rx.valid & (rx.data == 0xAA), NextState("TAG"))
        )
        fsm.act("TAG",
            rx.ready.eq(1),
            If(rx.valid,
                If(rx.data == 0x44,
                    NextValue(self.sof_pulse, 1),
                    NextState("SOF_CLR")
                ).Elif(rx.data == 0x66,
                    NextValue(self.eof_pulse, 1),
                    NextState("EOF_CLR")
                ).Elif(rx.data == 0x55,
                    NextState("EXPECT_R")
                ).Else(
                    NextState("WAIT_A")
                )
            )
        )
        fsm.act("SOF_CLR", NextValue(self.sof_pulse, 0), NextState("WAIT_A"))
        fsm.act("EOF_CLR", NextValue(self.eof_pulse, 0), NextState("WAIT_A"))

        fsm.act("EXPECT_R",
            rx.ready.eq(1),
            If(rx.valid,
                If(rx.data == ord('R'),
                    NextState("GET_Y0")
                ).Else(
                    NextState("WAIT_A")
                )
            )
        )
        fsm.act("GET_Y0",
            rx.ready.eq(1),
            If(rx.valid, NextValue(self.row_y[:8], rx.data), NextState("GET_Y1"))
        )
        fsm.act("GET_Y1",
            rx.ready.eq(1),
            If(rx.valid,
                NextValue(self.row_y[8:], rx.data),
                NextValue(words_out, 0),
                NextValue(have_lo, 0),
                NextState("PAYLOAD")
            )
        )
        fsm.act("PAYLOAD",
            If(words_out == H_VISIBLE,
                NextValue(self.row_done, 1),
                NextState("ROW_DONE_CLR")
            ).Else(
                If(~have_lo,
                    rx.ready.eq(1),
                    If(rx.valid, NextValue(low8, rx.data), NextValue(have_lo, 1))
                ).Else(
                    rx.ready.eq(dst.ready),
                    If(rx.valid & dst.ready,
                        dst.valid.eq(1),
                        dst.data.eq(Cat(low8, rx.data)),
                        NextValue(words_out, words_out + 1),
                        NextValue(have_lo, 0)
                    )
                )
            )
        )
        fsm.act("ROW_DONE_CLR", NextValue(self.row_done, 0), NextState("WAIT_A"))


# ---------------- Wishbone 32-bit row writer (sys) ----------------
class WB32RowWriter(LiteXModule):
    """Pulls 640x16b from src and writes 320x32b words to SDRAM at base + y*stride + widx."""
    def __init__(self, base_words, stride_words):
        self.src     = src = stream.Endpoint([("data",16)])
        self.wb      = wishbone.Interface()
        self.start   = Signal()
        self.y_line  = Signal(16)
        self.done    = Signal()

        word_idx = Signal(10)   # 0..319
        y_latch  = Signal(16)
        lo_hold  = Signal(16)

        adr_next = Signal(32)
        dat_next = Signal(32)

        fsm = FSM(reset_state="IDLE"); self.submodules += fsm
        fsm.act("IDLE",
            NextValue(self.done, 0),
            NextValue(word_idx, 0),
            If(self.start, NextValue(y_latch, self.y_line), NextState("GET_LO"))
        )
        fsm.act("GET_LO",
            src.ready.eq(1),
            If(src.valid, NextValue(lo_hold, src.data), NextState("GET_HI"))
        )
        fsm.act("GET_HI",
            src.ready.eq(1),
            If(src.valid,
                NextValue(dat_next, Cat(lo_hold, src.data)),         # LE pack
                NextValue(adr_next, base_words + stride_words*y_latch + word_idx),
                NextState("WRITE")
            )
        )
        fsm.act("WRITE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(1),
            self.wb.adr.eq(adr_next),
            self.wb.dat_w.eq(dat_next),
            self.wb.sel.eq(0xF),
            If(self.wb.ack,
                NextValue(word_idx, word_idx + 1),
                If(word_idx == (H_VISIBLE//2 - 1), NextState("DONE")).Else(NextState("GET_LO"))
            )
        )
        fsm.act("DONE", NextValue(self.done, 1), NextState("IDLE"))


# ---------------- Wishbone 32-bit line fetch -> BRAM (sys) ----------------
class WB32LineFetchToBRAM(LiteXModule):
    """Reads 320x32b words from front frame in SDRAM and writes 640x16b into a BRAM line buffer."""
    def __init__(self, base_words, stride_words):
        self.wb     = wishbone.Interface()
        self.start  = Signal()
        self.y_line = Signal(16)
        self.active = Signal()
        # BRAM write out
        self.out_we   = Signal()
        self.out_adr  = Signal(10)
        self.out_dat  = Signal(16)
        self.done     = Signal()

        word_idx = Signal(10)  # 0..319
        x        = Signal(10)  # 0..639
        y_latch  = Signal(16)
        data_lat = Signal(32)

        fsm = FSM(reset_state="IDLE"); self.submodules += fsm
        fsm.act("IDLE",
            NextValue(self.active, 0),
            NextValue(self.done, 0),
            NextValue(word_idx, 0),
            NextValue(x, 0),
            If(self.start, NextValue(self.active, 1), NextValue(y_latch, self.y_line), NextState("RD_ISSUE"))
        )
        fsm.act("RD_ISSUE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(0),
            self.wb.adr.eq(base_words + stride_words*y_latch + word_idx),
            If(self.wb.ack, NextValue(data_lat, self.wb.dat_r), NextState("WR_LO"))
        )
        fsm.act("WR_LO",
            self.out_we.eq(1), self.out_adr.eq(x), self.out_dat.eq(data_lat[:16]),
            NextValue(x, x + 1), NextState("WR_HI")
        )
        fsm.act("WR_HI",
            self.out_we.eq(1), self.out_adr.eq(x), self.out_dat.eq(data_lat[16:32]),
            NextValue(x, x + 1),
            If(word_idx == (H_VISIBLE//2 - 1), NextState("DONE")).Else(NextValue(word_idx, word_idx + 1), NextState("RD_ISSUE"))
        )
        fsm.act("DONE", NextValue(self.active, 0), NextValue(self.done, 1), NextState("IDLE"))


# ---------------- Simple frame CLEAR DMA (writes zeros) ----------------
class FrameClear(LiteXModule):
    """Clears a WxH 16bpp framebuffer in SDRAM to 0x0000 (black). Auto-starts once."""
    def __init__(self, base_words, stride_words, width=640, height=480):
        self.wb   = wishbone.Interface()
        self.done = Signal(reset=0)

        y     = Signal(16)
        widx  = Signal(10)          # 0..(width/2-1) -> 320 for 640

        fsm = FSM(reset_state="ROW0"); self.submodules += fsm
        fsm.act("ROW0",
            NextValue(y, 0),
            NextValue(widx, 0),
            NextValue(self.done, 0),
            NextState("ISSUE")
        )
        fsm.act("ISSUE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(1),
            self.wb.adr.eq(base_words + stride_words*y + widx),
            self.wb.dat_w.eq(0),                      # 2 pixels of 0x0000
            self.wb.sel.eq(0xF),
            If(self.wb.ack,
                If(widx == (width//2 - 1),
                    NextState("NEXT_ROW")
                ).Else(
                    NextValue(widx, widx + 1)
                )
            )
        )
        fsm.act("NEXT_ROW",
            NextValue(widx, 0),
            If(y == (height - 1),
                NextState("DONE")
            ).Else(
                NextValue(y, y + 1),
                NextState("ISSUE")
            )
        )
        fsm.act("DONE", NextValue(self.done, 1), NextState("DONE"))


# ---------------- Top SoC ----------------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=SYS_CLK_FREQ, **kwargs):
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0x8000)
        kwargs.setdefault("integrated_sram_size", 0x2000)
        kwargs.setdefault("with_uart", False)

        platform = sipeed_tang_nano_20k.Platform()
        try:
            SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, ident="UART->SDRAM Framebuffer->HDMI", **kwargs)
        except TypeError:
            SoCCore.__init__(self, platform, sys_clk_freq, ident="UART->SDRAM Framebuffer->HDMI", **kwargs)

        # Clocks
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SDRAM PHY wiring (drive SDRAM clock with DDROutput)
        sdram_pads = _SDRAMPads(platform)
        self.specials += DDROutput(0, 1, sdram_pads.clk, ClockSignal("sys"))
        self.submodules.sdrphy = GENSDRPHY(sdram_pads)   # gensdrphy expects just pads

        # Main SDRAM
        self.add_sdram("sdram",
            phy     = self.sdrphy,
            module  = M12L64322A(sys_clk_freq, "1:1"),
            l2_cache_size = 8192
        )

        main_ram_base = self.mem_map.get("main_ram", 0x4000_0000)
        WORDS_PER_LINE = (H_VISIBLE * 2) // 4        # 1280/4 = 320 words
        FRAME_WORDS    = WORDS_PER_LINE * V_VISIBLE  # 320*480 = 153600
        FRAME0_BASE    = (main_ram_base // 4)
        FRAME1_BASE    = FRAME0_BASE + FRAME_WORDS

        # HDMI PHY + timing
        pads = platform.request("hdmi")
        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(VideoGowinHDMIPHY(pads))
        self.submodules.tmg      = ClockDomainsRenamer("hdmi")(SimpleTimingHDMI())

        # Dual line BRAMs (A/B)
        memA = Memory(16, H_VISIBLE)
        memB = Memory(16, H_VISIBLE)
        self.specials += memA, memB
        rdA = memA.get_port(clock_domain="hdmi")
        rdB = memB.get_port(clock_domain="hdmi")
        wrA = memA.get_port(write_capable=True, clock_domain="sys")
        wrB = memB.get_port(write_capable=True, clock_domain="sys")
        self.specials += rdA, rdB, wrA, wrB

        # Read-bank (HDMI domain): 0->A, 1->B
        read_bank = Signal(reset=0)
        self.comb += [
            rdA.adr.eq(self.tmg.hcount[:10]),
            rdB.adr.eq(self.tmg.hcount[:10]),
        ]
        rd_pix = Signal(16)
        self.comb += rd_pix.eq(Mux(read_bank, rdB.dat_r, rdA.dat_r))

        # Display enable: high after front frame has been cleared (black),
        # or after first tear-free swap. Before that, output forced black.
        display_ready = Signal(reset=0)

        pix_sel = Signal(16)
        r8,g8,b8 = rgb565_to_888(pix_sel)
        self.comb += [
            pix_sel.eq(Mux(display_ready, rd_pix, 0)),  # black if not ready
            self.videophy.sink.valid.eq(1),
            self.videophy.sink.de.eq(self.tmg.de),
            self.videophy.sink.hsync.eq(self.tmg.hsync),
            self.videophy.sink.vsync.eq(self.tmg.vsync),
            self.videophy.sink.r.eq(r8),
            self.videophy.sink.g.eq(g8),
            self.videophy.sink.b.eq(b8),
        ]

        # --- HDMI DE edge detection ---
        de_d = Signal()
        self.sync.hdmi += de_d.eq(self.tmg.de)

        de_rise = Signal()
        de_fall = Signal()
        self.comb += [
            de_rise.eq(~de_d & self.tmg.de),   # start of visible line
            de_fall.eq(de_d & ~self.tmg.de),   # end of visible line
        ]

        # Flip read bank at end-of-line
        self.sync.hdmi += If(de_fall, read_bank.eq(~read_bank))

        # Next line index computed at DE rising (start-of-line) -> SYS
        y_next_hdmi = Signal(16)
        self.sync.hdmi += If(de_rise,
            y_next_hdmi.eq(Mux(self.tmg.vcount == (V_VISIBLE-1), 0, self.tmg.vcount + 1))
        )
        y_next_sys = Signal(16); self.specials += MultiReg(y_next_hdmi, y_next_sys, "sys")

        # Frame tick (VS rising) HDMI -> SYS
        vs_d = Signal()
        self.sync.hdmi += vs_d.eq(self.tmg.vsync)
        frame_tick_hdmi = Signal()
        self.comb += frame_tick_hdmi.eq(~vs_d & self.tmg.vsync)  # rising (active-low sync)
        frame_tick_sys = Signal(); self.specials += MultiReg(frame_tick_hdmi, frame_tick_sys, "sys")

        # UART PHY
        try:
            uart_pads = platform.request("custom_serial")
        except Exception:
            uart_pads = platform.request("serial")
        self.submodules.uartphy = UARTPHY(uart_pads, clk_freq=sys_clk_freq, baudrate=UART_BAUD)

        # Row FIFO and RX protocol
        self.submodules.row_fifo = SyncFIFO([("data",16)], depth=H_VISIBLE)  # holds one row
        self.submodules.rxproto  = ProtoUARTRx(self.uartphy.source, self.row_fifo.sink)

        # Selectable front/back SDRAM frame bases (word addresses)
        front_is_frame1 = Signal(reset=0)  # 0: front=FRAME0, 1: front=FRAME1
        front_base = Signal(32)
        back_base  = Signal(32)
        self.comb += [
            front_base.eq(Mux(front_is_frame1, FRAME1_BASE, FRAME0_BASE)),
            back_base.eq( Mux(front_is_frame1, FRAME0_BASE, FRAME1_BASE)),
        ]

        # DMAs
        self.submodules.row_writer = WB32RowWriter(base_words=back_base,  stride_words=WORDS_PER_LINE)
        self.submodules.line_fetch = WB32LineFetchToBRAM(base_words=front_base, stride_words=WORDS_PER_LINE)
        self.add_wb_master(self.row_writer.wb)
        self.add_wb_master(self.line_fetch.wb)

        # --- Clear both frames to BLACK at boot ---
        self.submodules.clear_front = FrameClear(base_words=FRAME0_BASE, stride_words=WORDS_PER_LINE,
                                                 width=H_VISIBLE, height=V_VISIBLE)
        self.submodules.clear_back  = FrameClear(base_words=FRAME1_BASE, stride_words=WORDS_PER_LINE,
                                                 width=H_VISIBLE, height=V_VISIBLE)
        self.add_wb_master(self.clear_front.wb)
        self.add_wb_master(self.clear_back.wb)

        # Enable display (and prefetch) only after the FRONT clear is done.
        self.sync += If(self.clear_front.done, display_ready.eq(1))

        # Write-bank (SYS) follows opposite of HDMI read bank (safely synced)
        read_bank_sys = Signal()
        self.specials += MultiReg(read_bank, read_bank_sys, "sys")
        write_bank = Signal()
        self.comb += write_bank.eq(~read_bank_sys)

        # Route line_fetch writes into the selected BRAM
        self.comb += [
            If(write_bank,
                wrB.we.eq(self.line_fetch.out_we),  wrB.adr.eq(self.line_fetch.out_adr),  wrB.dat_w.eq(self.line_fetch.out_dat),
                wrA.we.eq(0)
            ).Else(
                wrA.we.eq(self.line_fetch.out_we),  wrA.adr.eq(self.line_fetch.out_adr),  wrA.dat_w.eq(self.line_fetch.out_dat),
                wrB.we.eq(0)
            )
        ]

        # Kick a new line fetch at DE rising (sys) — but ONLY after display_ready
        de_rise_sys   = Signal(); self.specials += MultiReg(de_rise, de_rise_sys, "sys")
        de_rise_sys_d = Signal(); self.sync     += de_rise_sys_d.eq(de_rise_sys)
        line_kick     = Signal(); self.comb     += line_kick.eq(de_rise_sys & ~de_rise_sys_d)

        self.comb += [
            self.line_fetch.start.eq(line_kick & display_ready),
            self.line_fetch.y_line.eq(y_next_sys)
        ]

        # Handle rows: when RX finishes a row, kick the row writer and then send 'K'
        row_pending = Signal()
        row_y_latch = Signal(16)

        row_done_prev = Signal()
        row_done_edge = Signal()
        self.sync += [
            row_done_edge.eq(self.rxproto.row_done & ~row_done_prev),
            row_done_prev.eq(self.rxproto.row_done),
        ]

        self.sync += [
            If(row_done_edge & ~row_pending,
                row_pending.eq(1),
                row_y_latch.eq(self.rxproto.row_y)
            ).Elif(self.row_writer.done,
                row_pending.eq(0)
            )
        ]

        self.comb += [
            self.row_writer.start.eq(row_pending & ~self.row_writer.done),
            self.row_writer.y_line.eq(row_y_latch),
            self.row_writer.src.valid.eq(self.row_fifo.source.valid & row_pending),
            self.row_fifo.source.ready.eq(self.row_writer.src.ready & row_pending),
            self.row_writer.src.data.eq(self.row_fifo.source.data),
        ]

        # Ack TX FIFO -> UART (prioritize 'F' over 'K')
        self.submodules.ack_fifo = SyncFIFO([("data",8)], depth=8)
        self.comb += [
            self.uartphy.sink.valid.eq(self.ack_fifo.source.valid),
            self.uartphy.sink.data.eq(self.ack_fifo.source.data),
            self.ack_fifo.source.ready.eq(self.uartphy.sink.ready),
        ]

        # One-shot event generators
        roww_done_prev = Signal()
        k_edge = Signal()
        self.sync += [
            k_edge.eq(self.row_writer.done & ~roww_done_prev),
            roww_done_prev.eq(self.row_writer.done),
        ]

        # Frame swap request on EOF; perform at frame boundary (and only when no row DMA pending)
        swap_req = Signal()
        eof_prev = Signal()
        eof_edge = Signal()
        self.sync += [
            eof_edge.eq(self.rxproto.eof_pulse & ~eof_prev),
            eof_prev.eq(self.rxproto.eof_pulse),
            If(eof_edge, swap_req.eq(1))
        ]

        do_swap = Signal()
        self.sync += [
            do_swap.eq(0),
            If(frame_tick_sys & swap_req & ~row_pending,
                do_swap.eq(1),
                swap_req.eq(0)
            )
        ]
        self.sync += If(do_swap, front_is_frame1.eq(~front_is_frame1))

        # 'F' one-shot when we actually swap (also keeps display enabled)
        swap_prev = Signal()
        f_edge = Signal()
        self.sync += [
            f_edge.eq(do_swap & ~swap_prev),
            swap_prev.eq(do_swap),
            If(f_edge, display_ready.eq(1))
        ]

        # Enqueue acks (F has priority)
        ack_v    = Signal(reset=0)
        ack_data = Signal(8)
        self.comb += [
            self.ack_fifo.sink.valid.eq(ack_v),
            self.ack_fifo.sink.data.eq(ack_data),
        ]
        self.sync += [
            If(ack_v & self.ack_fifo.sink.ready,
                ack_v.eq(0)
            ).Elif(f_edge & ~ack_v,
                ack_v.eq(1), ack_data.eq(ord('F'))
            ).Elif(k_edge & ~ack_v,
                ack_v.eq(1), ack_data.eq(ord('K'))
            )
        ]

        # ---------------- LEDs / debug (active low LEDs on Tang Nano 20K) -------------
        def pulse(strobe, bits=21):
            cnt = Signal(bits); out = Signal()
            self.sync += If(strobe, cnt.eq((1<<bits)-1)).Elif(cnt != 0, cnt.eq(cnt-1))
            self.comb += out.eq(cnt != 0)
            return out

        try:
            led0 = platform.request("led_n", 0)
            led1 = platform.request("led_n", 1)
            led2 = platform.request("led_n", 2)
            led3 = platform.request("led_n", 3)
            led4 = platform.request("led_n", 4)
            led5 = platform.request("led_n", 5)

            # LED0: line_fetch active
            self.comb += led0.eq(~self.line_fetch.active)
            # LED1: row writer done pulses
            self.comb += led1.eq(~pulse(k_edge, 20))
            # LED2: UART TX byte
            tx_byte = Signal(); self.sync += tx_byte.eq(self.uartphy.sink.valid & self.uartphy.sink.ready)
            self.comb += led2.eq(~pulse(tx_byte, 21))
            # LED3: end-of-line (hdmi)
            self.comb += led3.eq(~pulse(de_fall, 22))
            # LED4/5: heartbeats (hdmi/sys)
            hb_hdmi = Signal(24); self.sync.hdmi += hb_hdmi.eq(hb_hdmi + 1)
            self.comb += led4.eq(~hb_hdmi[23])
            hb_sys  = Signal(26); self.sync      += hb_sys.eq(hb_sys + 1)
            self.comb += led5.eq(~hb_sys[25])
        except Exception:
            pass


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = parser.parse_args()

    soc  = Top()
    bld  = Builder(soc)
    if args.build:
        bld.build()
    if args.load:
        soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))


if __name__ == "__main__":
    main()
