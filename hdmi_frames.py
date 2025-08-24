#!/usr/bin/env python3
# Tang Nano 20K — UART → SDR SDRAM framebuffer → HDMI (640x480@60)
# Packet: 0xAA 0x55 'R'  y_lo y_hi  [1280 bytes RGB565 LE]

from migen import *
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.uart import UARTPHY
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.soc.interconnect import wishbone, stream
from litex.soc.integration.soc import SoCRegion
from litex.build.io import DDROutput

# your local platform (same one you used with the BaseSoC that builds)
from platforms import sipeed_tang_nano_20k

# SDRAM
from litedram.phy import GENSDRPHY
from litedram.modules import M12L64322A

# -------------------------- Video constants --------------------------
H_RES   = 640
V_RES   = 480
PIXCLK  = int(25_200_000)      # 640x480@60
PIXCLK5 = PIXCLK*5

BPP = 2                         # RGB565
LINE_BYTES      = H_RES*BPP
LINE_WORDS32    = (LINE_BYTES + 3)//4
FRAME_STRIDE_BYTES = ((LINE_BYTES + 31)//32)*32
FRAME_STRIDE_WORDS = FRAME_STRIDE_BYTES//4

HDMI_DOMAIN = "hdmi"
HDMI5_DOMAIN = "hdmi5x"

# ----------------------------- CRG -----------------------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(48e6)):
        self.cd_sys   = ClockDomain()
        self.cd_hdmi  = ClockDomain(HDMI_DOMAIN)
        self.cd_hdmi5x= ClockDomain(HDMI5_DOMAIN)

        clk27 = platform.request("clk27")

        # System PLL
        self.submodules.syspll = syspll = GW2APLL(devicename=platform.devicename, device=platform.device)
        syspll.register_clkin(clk27, 27e6)
        syspll.create_clkout(self.cd_sys, sys_clk_freq)

        # Video PLLs: 5x pixel and /5 divider (matches your BaseSoC style)
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIXCLK5, margin=1e-2)

        # Gowin CLKDIV /5 to pixel clock
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5",
            i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk,
            o_CLKOUT=self.cd_hdmi.clk
        )

# ---------------------- SDRAM pads wrapper ---------------------------
class _SDRAMPads:
    def __init__(self, platform):
        self.clk   = platform.request("O_sdram_clk")
        self.cke   = platform.request("O_sdram_cke")
        self.cs_n  = platform.request("O_sdram_cs_n")
        self.cas_n = platform.request("O_sdram_cas_n")
        self.ras_n = platform.request("O_sdram_ras_n")
        self.we_n  = platform.request("O_sdram_wen_n")
        self.dm    = platform.request("O_sdram_dqm")
        self.a     = platform.request("O_sdram_addr")
        self.ba    = platform.request("O_sdram_ba")
        self.dq    = platform.request("IO_sdram_dq")

# ---------------- UART row receiver -> stream of (y, 1280B) ---------
class RowRX(LiteXModule):
    """Parse AA 55 'R' y_lo y_hi + payload(1280B). Emits one Stream payload/row."""
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 8)])                     # from UARTPHY.source
        self.source = stream.Endpoint([("y", 16), ("data", 8)])          # row byte stream (y held constant)

        y_lo = Signal(8); y_hi = Signal(8)
        y    = Signal(16)
        count= Signal(11)  # 0..1279

        fsm = FSM(reset_state="WAIT_AA")
        self.submodules += fsm

        fsm.act("WAIT_AA",
            self.sink.ready.eq(1),
            If(self.sink.valid & (self.sink.data == 0xAA),
                NextState("WAIT_55")
            )
        )
        fsm.act("WAIT_55",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                If(self.sink.data == 0x55, NextState("WAIT_R")),
                If(self.sink.data != 0x55, NextState("WAIT_AA"))
            )
        )
        fsm.act("WAIT_R",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                If(self.sink.data == ord('R'), NextState("GET_YLO"))
                .Else(NextState("WAIT_AA"))
            )
        )
        fsm.act("GET_YLO",
            self.sink.ready.eq(1),
            If(self.sink.valid, NextValue(y_lo, self.sink.data), NextState("GET_YHI"))
        )
        fsm.act("GET_YHI",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                NextValue(y_hi, self.sink.data),
                NextValue(y, Cat(y_lo, self.sink.data)),
                NextValue(count, 0),
                NextState("PAYLOAD")
            )
        )
        fsm.act("PAYLOAD",
            self.sink.ready.eq(self.source.ready),
            self.source.valid.eq(self.sink.valid),
            self.source.data.eq(self.sink.data),
            self.source.y.eq(y),
            If(self.sink.valid & self.source.ready,
                If(count == (LINE_BYTES-1),
                    NextState("WAIT_AA")
                ).Else(
                    NextValue(count, count + 1)
                )
            )
        )

# ----------- Simple WB writer: writes one row (1280B) to SDRAM -------
class RowWBWriter(LiteXModule):
    """Consume RowRX bytes and perform 32-bit WB writes to SDRAM at base + y*stride."""
    def __init__(self, base_addr, stride_words):
        self.sink = stream.Endpoint([("y", 16), ("data", 8)])
        self.bus  = wishbone.Interface(data_width=32, adr_width=30)

        byte_cnt  = Signal(2)      # 0..3 within a 32-bit word
        wdata     = Signal(32)
        waddr     = Signal(len(self.bus.adr))
        y_lat     = Signal(16)
        words_in_row = Signal(16)

        row_base = Signal(len(self.bus.adr))
        self.comb += row_base.eq((base_addr>>2) + (y_lat * stride_words))

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        def wb_d():
            return [
                self.bus.cyc.eq(1),
                self.bus.stb.eq(1),
                self.bus.sel.eq(0xF),
                self.bus.we.eq(1),
                self.bus.adr.eq(row_base + waddr),
                self.bus.dat_w.eq(wdata)
            ]

        fsm.act("IDLE",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                NextValue(y_lat, self.sink.y),
                NextValue(byte_cnt, 0),
                NextValue(waddr, 0),
                NextValue(words_in_row, 0),
                NextValue(wdata, Cat(self.sink.data, Replicate(0, 24))),
                NextState("ACCUM")
            )
        )
        fsm.act("ACCUM",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                NextValue(wdata, (wdata >> 8) | (self.sink.data << 24)),
                If(byte_cnt == 3,
                    NextState("WRITE")
                ).Else(
                    NextValue(byte_cnt, byte_cnt + 1),
                    If(((words_in_row*4) + byte_cnt) == (LINE_BYTES-1),
                        NextValue(byte_cnt, 3),
                        NextState("WRITE")
                    )
                )
            )
        )
        fsm.act("WRITE",
            *wb_d(),
            If(self.bus.ack,
                NextValue(waddr, waddr + 1),
                NextValue(words_in_row, words_in_row + 1),
                If((words_in_row + 1) == LINE_WORDS32,
                    NextState("IDLE")
                ).Else(
                    NextState("ACCUM")
                )
            )
        )

# ------ Prefetch DMA: read one line into ping-pong RAM for HDMI ------
class LinePrefetchDMA(LiteXModule):
    """Given current scanline request, burst-read that line into BRAM (sys clk)."""
    def __init__(self, base_addr, stride_words, depth_words=LINE_WORDS32):
        self.bus   = wishbone.Interface(data_width=32, adr_width=30)  # from SDRAM
        self.req_y = Signal(16)   # set by video timing (sys domain)
        self.kick  = Signal()     # pulse to start prefetch for req_y

        # Ping-pong line RAM (sys write, hdmi read attaches externally)
        self.mem0  = Memory(32, depth_words)
        self.mem1  = Memory(32, depth_words)

        # sys write ports
        self.wr0   = self.mem0.get_port(write_capable=True, clock_domain="sys")
        self.wr1   = self.mem1.get_port(write_capable=True, clock_domain="sys")

        # NOTE: Gowin DPB doesn't support NO_CHANGE on Port1. Force READ_FIRST by
        # marking the *read* ports as transparent after creation.
        self.rd0   = self.mem0.get_port(clock_domain=HDMI_DOMAIN)
        self.rd1   = self.mem1.get_port(clock_domain=HDMI_DOMAIN)
        # Force READ_FIRST (no "no-change"), fixes PA2122.
        try:
            self.rd0.transparent = True
            self.rd1.transparent = True
        except Exception:
            pass

        self.specials += self.mem0, self.mem1, self.wr0, self.wr1, self.rd0, self.rd1

        # Which buffer we are filling
        self.buf_sel = Signal(reset=0)
        addr_w  = Signal(len(self.bus.adr))
        row_base= Signal(len(self.bus.adr))
        word_ix = Signal(max=depth_words)

        self.comb += row_base.eq((base_addr>>2) + (self.req_y * stride_words))

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        def wb_r():
            return [
                self.bus.cyc.eq(1),
                self.bus.stb.eq(1),
                self.bus.we.eq(0),
                self.bus.sel.eq(0xF),
                self.bus.adr.eq(addr_w)
            ]

        fsm.act("IDLE",
            If(self.kick,
                NextValue(addr_w, row_base),
                NextValue(word_ix, 0),
                NextState("READ")
            )
        )
        fsm.act("READ",
            *wb_r(),
            If(self.bus.ack,
                If(self.buf_sel == 0,
                    self.wr0.adr.eq(word_ix),
                    self.wr0.dat_w.eq(self.bus.dat_r),
                    self.wr0.we.eq(1)
                ).Else(
                    self.wr1.adr.eq(word_ix),
                    self.wr1.dat_w.eq(self.bus.dat_r),
                    self.wr1.we.eq(1)
                ),
                NextValue(word_ix, word_ix + 1),
                NextValue(addr_w, addr_w + 1),
                If(word_ix == (depth_words - 1),
                    NextValue(self.buf_sel, ~self.buf_sel),
                    NextState("IDLE")
                )
            )
        )

# ---------- HDMI pixel path: read ping-pong and spit RGB565 ----------
class LineReaderHDMI(LiteXModule):
    """HDMI domain reader: consumes the filled ping-pong buffers and outputs 8:8:8."""
    def __init__(self, dma, vtg):
        # Read current *DMA buf select* into HDMI domain. We’ll read the opposite one.
        sel_sys  = dma.buf_sel
        sel_hdmi = Signal()
        self.specials += MultiReg(sel_sys, sel_hdmi, HDMI_DOMAIN)

        word_ix  = Signal(max=LINE_WORDS32)
        byte_idx = Signal(2)   # 0..3 inside a word
        cur_word = Signal(32)

        # RGB to HDMI PHY
        self.r = Signal(8)
        self.g = Signal(8)
        self.b = Signal(8)

        # Only advance during active video
        with_de = Signal()
        self.comb += with_de.eq(vtg.source.de)

        # Address drive (HDMI domain)
        hdmi_sync = getattr(self.sync, HDMI_DOMAIN)
        hdmi_sync += [
            dma.rd0.adr.eq(word_ix),
            dma.rd1.adr.eq(word_ix),
        ]

        # choose the buffer NOT being filled next
        use0 = Signal()
        self.comb += use0.eq(sel_hdmi)   # if DMA toggled to fill #1 next, read #0 now

        # Fetch new word when we’ve output 4 bytes worth
        hdmi_sync += [
            If(~with_de,
                word_ix.eq(0),
                byte_idx.eq(0)
            ).Else(
                If(byte_idx == 0,
                    cur_word.eq(Mux(use0, dma.rd0.dat_r, dma.rd1.dat_r))
                ),
                byte_idx.eq(byte_idx + 1),
                If(byte_idx == 3,
                    byte_idx.eq(0),
                    word_ix.eq(word_ix + 1)
                )
            )
        ]

        # RGB565 -> 8:8:8 upsample
        pix16 = Signal(16)
        self.comb += [
            pix16.eq(Mux(byte_idx[0], (cur_word >> 8) & 0xFFFF, cur_word[0:16])),
            self.r.eq(Cat(Replicate(pix16[11:16], 1), pix16[11:16])),          # 5 -> 8
            self.g.eq(Cat(Replicate(pix16[5:11], 2), pix16[5:11][4:6])),       # 6 -> 8
            self.b.eq(Cat(Replicate(pix16[0:5], 3), pix16[0:5][2:5]))          # 5 -> 8
        ]

# ------------------------------ SoC ----------------------------------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(48e6), **kwargs):
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0)
        kwargs.setdefault("integrated_sram_size", 0x1000)
        kwargs.setdefault("with_uart", False)

        platform = sipeed_tang_nano_20k.Platform()

        # SoC
        try:
            SoCCore.__init__(self, platform,
                             clk_freq=sys_clk_freq,
                             ident="Tang Nano 20K — UART→SDRAM FB→HDMI",
                             **kwargs)
        except TypeError:
            SoCCore.__init__(self, platform, sys_clk_freq,
                             ident="Tang Nano 20K — UART→SDRAM FB→HDMI",
                             **kwargs)

        # Clocks
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SDR SDRAM PHY (and DDR clock out)
        pads = _SDRAMPads(platform)
        self.specials += DDROutput(0, 1, pads.clk, ClockSignal("sys"))
        self.submodules.sdrphy = GENSDRPHY(pads, sys_clk_freq)
        self.add_sdram("sdram",
            phy           = self.sdrphy,
            module        = M12L64322A(sys_clk_freq, "1:1"),
            l2_cache_size = 128
        )

        # Framebuffer base in SDRAM
        fb_base = self.bus.regions["main_ram"].origin
        self.fb_region = SoCRegion(origin=fb_base, size=FRAME_STRIDE_BYTES*V_RES, cached=True)

        # UART PHY @ sys clk
        self.submodules.uartphy = UARTPHY(
            pads     = platform.request("custom_serial"),
            clk_freq = sys_clk_freq,
            baudrate = 1_000_000
        )

        # Row receiver -> writer
        self.submodules.rowrx = RowRX()
        self.comb += [
            self.rowrx.sink.valid.eq(self.uartphy.source.valid),
            self.rowrx.sink.data.eq(self.uartphy.source.data),
            self.uartphy.source.ready.eq(self.rowrx.sink.ready),
        ]

        self.submodules.rowwr = RowWBWriter(base_addr=fb_base, stride_words=FRAME_STRIDE_WORDS)
        self.comb += [
            self.rowwr.sink.valid.eq(self.rowrx.source.valid),
            self.rowwr.sink.data.eq(self.rowrx.source.data),
            self.rowwr.sink.y.eq(self.rowrx.source.y),
            self.rowrx.source.ready.eq(self.rowwr.sink.ready),
        ]
        self.add_wb_master(self.rowwr.bus)

        # Video timing (HDMI domain)
        self.submodules.vtg = VideoTimingGenerator(default_video_timings="640x480@60Hz")

        # Prefetch DMA (sys), kick one line ahead of current vtg y
        self.submodules.linedma = LinePrefetchDMA(base_addr=fb_base, stride_words=FRAME_STRIDE_WORDS)
        self.add_wb_master(self.linedma.bus)

        # Mirror vtg.y into sys and prefetch next line
        y_hdmi = Signal(16)
        y_sys  = Signal(16)
        hdmi_sync = getattr(self.sync, HDMI_DOMAIN)
        hdmi_sync += y_hdmi.eq(self.vtg.source.vcount)
        self.specials += MultiReg(y_hdmi, y_sys, "sys")

        next_y = Signal(16)
        self.comb += [
            next_y.eq(Mux(y_sys == (V_RES-1), 0, y_sys + 1)),
            self.linedma.req_y.eq(next_y)
        ]
        prev_y = Signal(16)
        self.sync += prev_y.eq(y_sys)
        new_line = Signal()
        self.comb += new_line.eq(prev_y != y_sys)
        self.sync += [
            self.linedma.kick.eq(0),
            If(new_line, self.linedma.kick.eq(1))
        ]

        # HDMI pixel reader (hdmi domain)
        self.submodules.liner = LineReaderHDMI(self.linedma, self.vtg)

        # HDMI PHY (Gowin)
        hdmi_pads = platform.request("hdmi")
        self.submodules.hdmi_phy = VideoGowinHDMIPHY(hdmi_pads, clock_domain=HDMI_DOMAIN)

        # Drive PHY with timing + RGB
        self.comb += [
            self.hdmi_phy.sink.valid.eq(self.vtg.source.valid),
            self.hdmi_phy.sink.de.eq(self.vtg.source.de),
            self.hdmi_phy.sink.hsync.eq(self.vtg.source.hsync),
            self.hdmi_phy.sink.vsync.eq(self.vtg.source.vsync),
            self.hdmi_phy.sink.r.eq(self.liner.r),
            self.hdmi_phy.sink.g.eq(self.liner.g),
            self.hdmi_phy.sink.b.eq(self.liner.b),
        ]

        # Simple heartbeats on LEDs
        leds = platform.request_all("led_n")
        hb_sys  = Signal(26)
        hb_hdmi = Signal(24)
        self.sync     += hb_sys.eq(hb_sys + 1)
        hdmi_sync     += hb_hdmi.eq(hb_hdmi + 1)
        if len(leds) >= 2:
            self.comb += [
                leds[0].eq(~hb_sys[25]),
                leds[1].eq(~hb_hdmi[23]),
            ]

# --------------------------- Build/Load ------------------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform,
        description="Tang Nano 20K — UART→SDR SDRAM framebuffer→HDMI (640x480 RGB565)")
    p.add_target_argument("--sys-clk-freq", default=48e6,  type=float)
    args = p.parse_args()

    soc = Top(sys_clk_freq=int(args.sys_clk_freq), **p.soc_argdict)
    bld = Builder(soc, **p.builder_argdict)

    if args.build:
        bld.build(**p.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        try:
            bit = bld.get_bitstream_filename(mode="sram")
        except TypeError:
            bit = bld.get_bitstream_filename()
        prog.load_bitstream(bit)

if __name__ == "__main__":
    main()
