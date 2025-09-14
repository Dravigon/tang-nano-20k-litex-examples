#!/usr/bin/env python3
# Tang Nano 20K – SPI -> SDRAM framebuffer (double-buffered) -> HDMI 640x480@60
# True 8-bit RGB332 color format: RRRGGGBB

from migen import *
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule, ClockDomainsRenamer
from litex.build.io import DDROutput
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.interconnect import wishbone, stream
from litex.soc.interconnect.stream import SyncFIFO
from litedram.phy import GENSDRPHY
from litedram.modules import M12L64322A

try:
    from platforms import sipeed_tang_nano_20k
except Exception:
    from litex_boards.platforms import sipeed_tang_nano_20k

# Game resolution: 320x200 (classic Doom), scaled 2x to 640x480 HDMI
GAME_WIDTH   = 320
GAME_HEIGHT  = 200

# HDMI output timing: 640x480@60 (for display compatibility)
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
SYS_CLK_FREQ = int(48e6)

# Scaling factors for 320x200 -> 640x480
H_SCALE     = H_VISIBLE // GAME_WIDTH  # 2x horizontal
V_SCALE     = V_VISIBLE // GAME_HEIGHT  # 2x vertical (with centering)

# SDRAM pads wrapper
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

# Clock generation
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=SYS_CLK_FREQ):
        self.cd_sys    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()
        clk27 = platform.request("clk27")
        
        self.submodules.syspll = syspll = GW2APLL(devicename=platform.devicename, device=platform.device)
        syspll.register_clkin(clk27, 27e6)
        syspll.create_clkout(self.cd_sys, sys_clk_freq)
        
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIX5X, margin=1e-2)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5", i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk, o_CLKOUT=self.cd_hdmi.clk
        )

# HDMI timing generator (still 640x480 output)
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
            self.hsync.eq(~((self.hcount >= HS_BEG) & (self.hcount < HS_END))),
            self.vsync.eq(~((self.vcount >= VS_BEG) & (self.vcount < VS_END))),
        ]

# Clean SPI Slave with working TX/RX
class SPISlaveEcho_MODE0_SYS(LiteXModule):
    def __init__(self, pads):
        self.source = stream.Endpoint([("data", 8)])
        self.sink = stream.Endpoint([("data", 8)])
        
        sck_i = pads.clk if hasattr(pads, "clk") else pads.sck
        mosi_i = pads.mosi
        miso_o = pads.miso
        csn_i = pads.cs_n
        
        # Define all signals first
        sck_m = Signal(); sck = Signal(); sck_d = Signal()
        mosi_m = Signal(); mosi = Signal()
        cs_m = Signal(reset=1); cs = Signal(reset=1); cs_d = Signal(reset=1)
        bitcnt = Signal(3)
        rx_shift = Signal(8)
        tx_shift = Signal(8)
        miso_q = Signal(reset=1)
        tx_data = Signal(8, reset=0xA5)
        tx_ready = Signal(reset=1)
        rx_data = Signal(8)
        rx_valid = Signal()
        
        # Sync inputs
        self.sync += [
            sck_m.eq(sck_i), sck.eq(sck_m), sck_d.eq(sck),
            mosi_m.eq(mosi_i), mosi.eq(mosi_m),
            cs_m.eq(csn_i), cs.eq(cs_m), cs_d.eq(cs),
        ]
        
        cs_active = ~cs
        sck_rise = ~sck_d & sck
        sck_fall = sck_d & ~sck
        cs_fall = cs_d & ~cs
        cs_rise = ~cs_d & cs
        
        # TX data handling
        self.sync += [
            If(self.sink.valid & self.sink.ready,
                tx_data.eq(self.sink.data),
                tx_ready.eq(0)
            ).Elif(cs_fall | (cs_active & (bitcnt == 0) & sck_fall),
                tx_ready.eq(1)
            )
        ]
        self.comb += self.sink.ready.eq(tx_ready)
        
        # Main SPI logic
        self.sync += [
            If(cs_fall,
                bitcnt.eq(0),
                rx_shift.eq(0),
                tx_shift.eq(tx_data),
                miso_q.eq(tx_data[7]),
                rx_valid.eq(0)
            ).Elif(cs_rise,
                bitcnt.eq(0),
                rx_valid.eq(0)
            ),
            
            If(cs_active,
                If(sck_rise,
                    rx_shift.eq(Cat(mosi, rx_shift[:-1])),
                    If(bitcnt == 7,
                        rx_data.eq(Cat(mosi, rx_shift[:-1])),
                        rx_valid.eq(1),
                        bitcnt.eq(0)
                    ).Else(
                        bitcnt.eq(bitcnt + 1),
                        rx_valid.eq(0)
                    )
                ).Else(
                    rx_valid.eq(0)
                ),
                
                If(sck_fall,
                    If(bitcnt == 0,
                        tx_shift.eq(tx_data),
                        miso_q.eq(tx_data[7])
                    ).Else(
                        miso_q.eq(tx_shift[6]),
                        tx_shift.eq(Cat(Constant(0,1), tx_shift[:-1]))
                    )
                )
            ).Else(
                rx_valid.eq(0)
            )
        ]
        
        self.comb += [
            miso_o.eq(Mux(cs_active, miso_q, 1)),
            self.source.valid.eq(rx_valid),
            self.source.data.eq(rx_data)
        ]

# Protocol receiver - RGB332 format
class ProtoSPIRx(LiteXModule):
    def __init__(self, spi_source, row_fifo_sink):
        rx = spi_source
        dst = row_fifo_sink
        
        self.row_y = Signal(16)
        self.row_done = Signal()
        self.sof_pulse = Signal()
        self.eof_pulse = Signal()
        
        pixels_out = Signal(10)  # Count pixels
        
        self.comb += [dst.valid.eq(0), dst.data.eq(0), rx.ready.eq(0)]
        
        fsm = FSM(reset_state="WAIT_A")
        self.submodules += fsm
        
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
                NextValue(pixels_out, 0),
                NextState("PAYLOAD")
            )
        )
        fsm.act("PAYLOAD",
            If(pixels_out == GAME_WIDTH,  # 320 pixels per row
                NextValue(self.row_done, 1),
                NextState("ROW_DONE_CLR")
            ).Else(
                rx.ready.eq(dst.ready),
                If(rx.valid & dst.ready,
                    dst.valid.eq(1),
                    dst.data.eq(rx.data),  # Direct RGB332 data
                    NextValue(pixels_out, pixels_out + 1)
                )
            )
        )
        fsm.act("ROW_DONE_CLR", NextValue(self.row_done, 0), NextState("WAIT_A"))

# Row writer to SDRAM - RGB332 data
class WB8RowWriter(LiteXModule):
    def __init__(self, base_words, stride_words):
        self.src = src = stream.Endpoint([("data", 8)])  # 8-bit RGB332 pixels
        self.wb = wishbone.Interface()
        self.start = Signal()
        self.y_line = Signal(16)
        self.done = Signal()
        
        pixel_idx = Signal(10)
        y_latch = Signal(16)
        pixel_buffer = Signal(32)  # Pack 4 pixels into 32-bit word
        pixel_count = Signal(2)    # 0-3 pixels in buffer
        adr_next = Signal(32)
        
        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        
        fsm.act("IDLE",
            NextValue(self.done, 0),
            NextValue(pixel_idx, 0),
            NextValue(pixel_count, 0),
            NextValue(pixel_buffer, 0),
            If(self.start, NextValue(y_latch, self.y_line), NextState("GET_PIXEL"))
        )
        fsm.act("GET_PIXEL",
            src.ready.eq(1),
            If(src.valid,
                # Pack pixel into buffer
                If(pixel_count == 0,
                    NextValue(pixel_buffer[0:8], src.data),
                    NextValue(pixel_count, 1)
                ).Elif(pixel_count == 1,
                    NextValue(pixel_buffer[8:16], src.data),
                    NextValue(pixel_count, 2)
                ).Elif(pixel_count == 2,
                    NextValue(pixel_buffer[16:24], src.data),
                    NextValue(pixel_count, 3)
                ).Else(  # pixel_count == 3
                    NextValue(pixel_buffer[24:32], src.data),
                    NextValue(adr_next, base_words + stride_words*y_latch + (pixel_idx >> 2)),
                    NextState("WRITE")
                ),
                NextValue(pixel_idx, pixel_idx + 1)
            )
        )
        fsm.act("WRITE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(1),
            self.wb.adr.eq(adr_next),
            self.wb.dat_w.eq(pixel_buffer),
            self.wb.sel.eq(0xF),
            If(self.wb.ack,
                If(pixel_idx == GAME_WIDTH,
                    # Handle remaining pixels in buffer
                    If(pixel_count == 0, NextState("DONE")).Else(NextState("FLUSH_REMAINING"))
                ).Else(
                    NextValue(pixel_count, 0),
                    NextValue(pixel_buffer, 0),
                    NextState("GET_PIXEL")
                )
            )
        )
        fsm.act("FLUSH_REMAINING",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(1),
            self.wb.adr.eq(adr_next),
            self.wb.dat_w.eq(pixel_buffer),
            self.wb.sel.eq(Mux(pixel_count == 1, 0x1,
                           Mux(pixel_count == 2, 0x3, 0x7))),  # Partial write
            If(self.wb.ack, NextState("DONE"))
        )
        fsm.act("DONE", NextValue(self.done, 1), NextState("IDLE"))

# Line fetcher from SDRAM - RGB332 data
class WB8LineFetchToBRAM(LiteXModule):
    def __init__(self, base_words, stride_words):
        self.wb = wishbone.Interface()
        self.start = Signal()
        self.y_line = Signal(16)
        self.active = Signal()
        self.out_we = Signal()
        self.out_adr = Signal(9)   # 320 pixels = 512 max (9 bits)
        self.out_dat = Signal(8)   # 8-bit RGB332 pixel
        self.done = Signal()
        
        word_idx = Signal(8)     # 320/4 = 80 words per line
        pixel_in_word = Signal(2) # 0-3 pixels per 32-bit word
        x = Signal(9)
        y_latch = Signal(16)
        data_lat = Signal(32)
        
        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        
        fsm.act("IDLE",
            NextValue(self.active, 0),
            NextValue(self.done, 0),
            NextValue(word_idx, 0),
            NextValue(pixel_in_word, 0),
            NextValue(x, 0),
            If(self.start, NextValue(self.active, 1), NextValue(y_latch, self.y_line), NextState("RD_ISSUE"))
        )
        fsm.act("RD_ISSUE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(0),
            self.wb.adr.eq(base_words + stride_words*y_latch + word_idx),
            If(self.wb.ack, NextValue(data_lat, self.wb.dat_r), NextState("WR_PIXELS"))
        )
        fsm.act("WR_PIXELS",
            self.out_we.eq(1),
            self.out_adr.eq(x),
            # Extract pixel from 32-bit word
            If(pixel_in_word == 0,
                self.out_dat.eq(data_lat[0:8])
            ).Elif(pixel_in_word == 1,
                self.out_dat.eq(data_lat[8:16])
            ).Elif(pixel_in_word == 2,
                self.out_dat.eq(data_lat[16:24])
            ).Else(
                self.out_dat.eq(data_lat[24:32])
            ),
            NextValue(x, x + 1),
            If(pixel_in_word == 3,
                NextValue(pixel_in_word, 0),
                If(x == (GAME_WIDTH - 1), NextState("DONE")).Else(NextValue(word_idx, word_idx + 1), NextState("RD_ISSUE"))
            ).Else(
                NextValue(pixel_in_word, pixel_in_word + 1),
                If(x == (GAME_WIDTH - 1), NextState("DONE"))
            )
        )
        fsm.act("DONE", NextValue(self.active, 0), NextValue(self.done, 1), NextState("IDLE"))

# Frame clear - for game resolution
class FrameClear(LiteXModule):
    def __init__(self, base_words, stride_words, width=GAME_WIDTH, height=GAME_HEIGHT):
        self.wb = wishbone.Interface()
        self.done = Signal(reset=0)
        
        y = Signal(16)
        widx = Signal(8)  # Fewer words needed for 320 pixels
        
        fsm = FSM(reset_state="ROW0")
        self.submodules += fsm
        
        fsm.act("ROW0",
            NextValue(y, 0),
            NextValue(widx, 0),
            NextValue(self.done, 0),
            NextState("ISSUE")
        )
        fsm.act("ISSUE",
            self.wb.cyc.eq(1), self.wb.stb.eq(1), self.wb.we.eq(1),
            self.wb.adr.eq(base_words + stride_words*y + widx),
            self.wb.dat_w.eq(0),
            self.wb.sel.eq(0xF),
            If(self.wb.ack,
                If(widx == ((width + 3) // 4 - 1),  # Round up to word boundary
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

# RGB332 to RGB888 expansion module
class RGB332Decoder(LiteXModule):
    def __init__(self):
        self.rgb332_in = Signal(8)   # Input: RRRGGGBB
        self.r_out = Signal(8)       # Output: 8-bit red
        self.g_out = Signal(8)       # Output: 8-bit green
        self.b_out = Signal(8)       # Output: 8-bit blue
        
        # Extract components from RGB332
        r3 = Signal(3)  # 3-bit red
        g3 = Signal(3)  # 3-bit green
        b2 = Signal(2)  # 2-bit blue
        
        self.comb += [
            r3.eq(self.rgb332_in[5:8]),  # Bits 7-5
            g3.eq(self.rgb332_in[2:5]),  # Bits 4-2
            b2.eq(self.rgb332_in[0:2]),  # Bits 1-0
            
            # Expand to 8-bit by replicating bits
            # Red: 3 bits to 8 bits (RRR -> RRR RRR RR)
            self.r_out.eq(Cat(r3[0:2], r3, r3)),
            
            # Green: 3 bits to 8 bits (GGG -> GGG GGG GG)
            self.g_out.eq(Cat(g3[0:2], g3, g3)),
            
            # Blue: 2 bits to 8 bits (BB -> BB BB BB BB)
            self.b_out.eq(Cat(b2, b2, b2, b2))
        ]

# Main SoC - Modified for RGB332
class Top(SoCCore):
    def __init__(self, sys_clk_freq=SYS_CLK_FREQ, **kwargs):
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0x8000)
        kwargs.setdefault("integrated_sram_size", 0x2000)
        kwargs.setdefault("with_uart", False)
        
        platform = sipeed_tang_nano_20k.Platform()
        try:
            SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, ident="SPI->SDRAM->HDMI-RGB332", **kwargs)
        except TypeError:
            SoCCore.__init__(self, platform, sys_clk_freq, ident="SPI->SDRAM->HDMI-RGB332", **kwargs)
        
        self.submodules.crg = _CRG(platform, sys_clk_freq)
        
        # SDRAM
        sdram_pads = _SDRAMPads(platform)
        self.specials += DDROutput(0, 1, sdram_pads.clk, ClockSignal("sys"))
        self.submodules.sdrphy = GENSDRPHY(sdram_pads)
        self.add_sdram("sdram",
            phy = self.sdrphy,
            module = M12L64322A(sys_clk_freq, "1:1"),
            l2_cache_size = 8192
        )
        
        main_ram_base = self.mem_map.get("main_ram", 0x4000_0000)
        # 8-bit RGB332 pixels, 320 pixels per line
        BYTES_PER_LINE = GAME_WIDTH  # 320 bytes per line
        WORDS_PER_LINE = (BYTES_PER_LINE + 3) // 4  # Round up to 32-bit words
        FRAME_WORDS = WORDS_PER_LINE * GAME_HEIGHT  # 80 * 200 = 16,000 words per frame
        FRAME0_BASE = (main_ram_base // 4)
        FRAME1_BASE = FRAME0_BASE + FRAME_WORDS
        
        # HDMI
        pads = platform.request("hdmi")
        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(VideoGowinHDMIPHY(pads))
        self.submodules.tmg = ClockDomainsRenamer("hdmi")(SimpleTimingHDMI())
        
        # Line buffers - 8-bit RGB332 pixels
        memA = Memory(8, GAME_WIDTH)  # 8-bit RGB332 pixels
        memB = Memory(8, GAME_WIDTH)
        self.specials += memA, memB
        rdA = memA.get_port(clock_domain="hdmi")
        rdB = memB.get_port(clock_domain="hdmi")
        wrA = memA.get_port(write_capable=True, clock_domain="sys")
        wrB = memB.get_port(write_capable=True, clock_domain="sys")
        self.specials += rdA, rdB, wrA, wrB
        
        # RGB332 decoder for direct color interpretation
        self.submodules.rgb_decoder = ClockDomainsRenamer("hdmi")(RGB332Decoder())
        
        read_bank = Signal(reset=0)
        
        # Scaling logic for 320x200 -> 640x480
        game_x = Signal(9)  # 0-319
        game_y = Signal(8)  # 0-199
        
        # Map HDMI coordinates to game coordinates  
        self.comb += [
            game_x.eq(self.tmg.hcount >> 1),  # Divide by 2 for 2x scaling
            game_y.eq((self.tmg.vcount - 40) >> 1),  # Center vertically and scale
        ]
        
        # Address line buffers with scaled coordinates
        valid_game_area = (self.tmg.hcount < 640) & (self.tmg.vcount >= 40) & (self.tmg.vcount < 440)
        self.comb += [
            rdA.adr.eq(game_x),
            rdB.adr.eq(game_x),
        ]
        
        rgb332_pixel = Signal(8)
        self.comb += rgb332_pixel.eq(Mux(read_bank, rdB.dat_r, rdA.dat_r))
        
        # Direct RGB332 to RGB888 conversion
        self.comb += self.rgb_decoder.rgb332_in.eq(rgb332_pixel)
        
        display_ready = Signal(reset=0)
        self.comb += [
            self.videophy.sink.valid.eq(1),
            self.videophy.sink.de.eq(self.tmg.de),
            self.videophy.sink.hsync.eq(self.tmg.hsync),
            self.videophy.sink.vsync.eq(self.tmg.vsync),
            self.videophy.sink.r.eq(Mux(display_ready & valid_game_area, self.rgb_decoder.r_out, 0)),
            self.videophy.sink.g.eq(Mux(display_ready & valid_game_area, self.rgb_decoder.g_out, 0)),
            self.videophy.sink.b.eq(Mux(display_ready & valid_game_area, self.rgb_decoder.b_out, 0)),
        ]
        
        # DE edge detection
        de_d = Signal()
        self.sync.hdmi += de_d.eq(self.tmg.de)
        de_rise = Signal()
        de_fall = Signal()
        self.comb += [
            de_rise.eq(~de_d & self.tmg.de),
            de_fall.eq(de_d & ~self.tmg.de),
        ]
        self.sync.hdmi += If(de_fall, read_bank.eq(~read_bank))
        
        y_next_hdmi = Signal(16)
        self.sync.hdmi += If(de_rise,
            y_next_hdmi.eq(Mux(game_y == (GAME_HEIGHT-1), 0, game_y + 1))
        )
        y_next_sys = Signal(16)
        self.specials += MultiReg(y_next_hdmi, y_next_sys, "sys")
        
        vs_d = Signal()
        self.sync.hdmi += vs_d.eq(self.tmg.vsync)
        frame_tick_hdmi = Signal()
        self.comb += frame_tick_hdmi.eq(~vs_d & self.tmg.vsync)
        frame_tick_sys = Signal()
        self.specials += MultiReg(frame_tick_hdmi, frame_tick_sys, "sys")
        
        # SPI
        try:
            spi_pads = platform.request("custom_spi", 0)
        except Exception:
            spi_pads = platform.request("spi")
        self.submodules.spi = SPISlaveEcho_MODE0_SYS(spi_pads)
        
        # Buffering
        self.submodules.spi_byte_fifo = SyncFIFO([("data",8)], depth=64)
        self.comb += [
            self.spi_byte_fifo.sink.valid.eq(self.spi.source.valid),
            self.spi_byte_fifo.sink.data.eq(self.spi.source.data),
            self.spi.source.ready.eq(self.spi_byte_fifo.sink.ready),
        ]
        
        self.submodules.row_fifo = SyncFIFO([("data",8)], depth=GAME_WIDTH)  # 8-bit RGB332 data
        self.submodules.rxproto = ProtoSPIRx(self.spi_byte_fifo.source, self.row_fifo.sink)
        
        # Frame management
        front_is_frame1 = Signal(reset=0)
        front_base = Signal(32)
        back_base = Signal(32)
        self.comb += [
            front_base.eq(Mux(front_is_frame1, FRAME1_BASE, FRAME0_BASE)),
            back_base.eq(Mux(front_is_frame1, FRAME0_BASE, FRAME1_BASE)),
        ]
        
        # DMA - for RGB332 data
        self.submodules.row_writer = WB8RowWriter(base_words=back_base, stride_words=WORDS_PER_LINE)
        self.submodules.line_fetch = WB8LineFetchToBRAM(base_words=front_base, stride_words=WORDS_PER_LINE)
        self.add_wb_master(self.row_writer.wb)
        self.add_wb_master(self.line_fetch.wb)
        
        # Clear frames
        self.submodules.clear_front = FrameClear(base_words=FRAME0_BASE, stride_words=WORDS_PER_LINE,
                                                 width=GAME_WIDTH, height=GAME_HEIGHT)
        self.submodules.clear_back = FrameClear(base_words=FRAME1_BASE, stride_words=WORDS_PER_LINE,
                                                width=GAME_WIDTH, height=GAME_HEIGHT)
        self.add_wb_master(self.clear_front.wb)
        self.add_wb_master(self.clear_back.wb)
        
        self.sync += If(self.clear_front.done, display_ready.eq(1))
        
        # Bank switching
        read_bank_sys = Signal()
        self.specials += MultiReg(read_bank, read_bank_sys, "sys")
        write_bank = Signal()
        self.comb += write_bank.eq(~read_bank_sys)
        
        self.comb += [
            If(write_bank,
                wrB.we.eq(self.line_fetch.out_we),
                wrB.adr.eq(self.line_fetch.out_adr),
                wrB.dat_w.eq(self.line_fetch.out_dat),
                wrA.we.eq(0)
            ).Else(
                wrA.we.eq(self.line_fetch.out_we),
                wrA.adr.eq(self.line_fetch.out_adr),
                wrA.dat_w.eq(self.line_fetch.out_dat),
                wrB.we.eq(0)
            )
        ]
        
        de_rise_sys = Signal()
        self.specials += MultiReg(de_rise, de_rise_sys, "sys")
        de_rise_sys_d = Signal()
        self.sync += de_rise_sys_d.eq(de_rise_sys)
        line_kick = Signal()
        self.comb += line_kick.eq(de_rise_sys & ~de_rise_sys_d)
        
        self.comb += [
            self.line_fetch.start.eq(line_kick & display_ready),
            self.line_fetch.y_line.eq(y_next_sys)
        ]
        
        # Row handling
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
        
        # Acknowledgments
        self.submodules.ack_fifo = SyncFIFO([("data",8)], depth=8)
        self.comb += [
            self.spi.sink.valid.eq(self.ack_fifo.source.valid),
            self.spi.sink.data.eq(self.ack_fifo.source.data),
            self.ack_fifo.source.ready.eq(self.spi.sink.ready),
        ]
        
        roww_done_prev = Signal()
        k_edge = Signal()
        self.sync += [
            k_edge.eq(self.row_writer.done & ~roww_done_prev),
            roww_done_prev.eq(self.row_writer.done),
        ]
        
        # Frame swap with immediate EOF ACK
        swap_req = Signal()
        eof_prev = Signal()
        eof_edge = Signal()
        eof_ack_pending = Signal()
        
        self.sync += [
            eof_edge.eq(self.rxproto.eof_pulse & ~eof_prev),
            eof_prev.eq(self.rxproto.eof_pulse),
            If(eof_edge, 
                swap_req.eq(1),
                eof_ack_pending.eq(1)  # Set ACK pending immediately
            )
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
        
        swap_prev = Signal()
        f_edge = Signal()
        self.sync += [
            f_edge.eq(do_swap & ~swap_prev),
            swap_prev.eq(do_swap),
            If(f_edge, display_ready.eq(1))
        ]
        
        # Enhanced ACK system with immediate EOF response
        ack_v = Signal(reset=0)
        ack_data = Signal(8)
        self.comb += [
            self.ack_fifo.sink.valid.eq(ack_v),
            self.ack_fifo.sink.data.eq(ack_data),
        ]
        
        self.sync += [
            If(ack_v & self.ack_fifo.sink.ready,
                ack_v.eq(0),
                # Clear EOF ACK pending when 'F' is sent
                If(ack_data == ord('F'), eof_ack_pending.eq(0))
            ).Elif(eof_ack_pending & ~ack_v,  # PRIORITY: Send 'F' immediately
                ack_v.eq(1), 
                ack_data.eq(ord('F'))
            ).Elif(k_edge & ~ack_v,  # Send 'K' for row complete
                ack_v.eq(1), 
                ack_data.eq(ord('K'))
            )
        ]
        
        # LEDs
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
            
            self.comb += [
                led0.eq(~(self.clear_front.done & self.clear_back.done)),
                led1.eq(~pulse(row_done_edge, 20)),
                led2.eq(~pulse(self.spi.source.valid & self.spi.source.ready, 21)),
                led3.eq(~pulse(self.rxproto.sof_pulse | self.rxproto.eof_pulse, 22)),
                led4.eq(~(row_pending & ~self.row_writer.done)),
                led5.eq(~display_ready)
            ]
        except Exception:
            pass

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = parser.parse_args()
    
    soc = Top()
    bld = Builder(soc)
    if args.build:
        bld.build()
    if args.load:
        soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()