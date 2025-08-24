#!/usr/bin/env python3
# Tang Nano 20K — HDMI color bars using a single Wishbone.SRAM (double line-buffer + simple HDMI timing)
# LEDs (active-low):
#  LED0: sys heartbeat
#  LED1: hdmi heartbeat
#  LED2: underflow flag (ON=OK, OFF=underflow this frame)
#  LED3: DE rising edge activity (blinks if scanlines happen)
#  LED4: pulses when a row finished pushing from SRAM -> FIFO (blinks if writer runs)

from migen import *
from migen.genlib.cdc import MultiReg, ClockDomainsRenamer
from litex.gen import LiteXModule
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.stream import AsyncFIFO

from platforms import sipeed_tang_nano_20k

# ---------------------- Video/geometry ----------------------
# 640x480@60 CVT-ish (classic VGA):
H_VISIBLE   = 640
V_VISIBLE   = 480
H_FP        = 16
H_SYNC      = 96
H_BP        = 48
H_TOTAL     = H_VISIBLE + H_FP + H_SYNC + H_BP   # 800

V_FP        = 10
V_SYNC      = 2
V_BP        = 33
V_TOTAL     = V_VISIBLE + V_FP + V_SYNC + V_BP   # 525

PIXCLK      = 25_200_000           # works well on Tang Nano 20K
PIX5X       = PIXCLK*5

BPP              = 16              # RGB565
LINE_BYTES       = H_VISIBLE * (BPP//8)          # 1280
LINE_WORDS32     = (LINE_BYTES + 3)//4           # 320
SRAM_BYTES       = 2 * LINE_WORDS32 * 4          # ~2.5 KiB double-buffer

# ----------------------------- CRG -----------------------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(48e6)):
        self.cd_sys    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")

        # System PLL
        self.submodules.syspll = syspll = GW2APLL(devicename=platform.devicename, device=platform.device)
        syspll.register_clkin(clk27, 27e6)
        syspll.create_clkout(self.cd_sys, sys_clk_freq)

        # Video PLLs: 5x pixel then /5
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIX5X, margin=1e-2)

        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5",
            i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk,
            o_CLKOUT=self.cd_hdmi.clk
        )

# --------- RGB565 -> 8:8:8 ----------
def rgb565_to_888(pix16):
    r5 = pix16[11:16]; g6 = pix16[5:11]; b5 = pix16[0:5]
    r = Cat(r5, r5[2:5])       # 5->8
    g = Cat(g6, g6[4:6])       # 6->8
    b = Cat(b5, b5[2:5])       # 5->8
    return r, g, b

# --------- Minimal HDMI timing (HDMI domain) ----------
class SimpleTimingHDMI(LiteXModule):
    def __init__(self):
        self.de     = Signal()
        self.hsync  = Signal()
        self.vsync  = Signal()
        self.valid  = Signal(reset=1)
        self.hcount = Signal(16)
        self.vcount = Signal(16)

        # HSYNC/VSYNC are negative polarity for 640x480@60
        HS_BEG = H_VISIBLE + H_FP
        HS_END = H_VISIBLE + H_FP + H_SYNC

        VS_BEG = V_VISIBLE + V_FP
        VS_END = V_VISIBLE + V_FP + V_SYNC

        self.sync.hdmi += [
            # h counter
            If(self.hcount == (H_TOTAL - 1),
                self.hcount.eq(0),
                # v counter
                If(self.vcount == (V_TOTAL - 1),
                    self.vcount.eq(0)
                ).Else(
                    self.vcount.eq(self.vcount + 1)
                )
            ).Else(
                self.hcount.eq(self.hcount + 1)
            ),

            # DE
            self.de.eq((self.hcount < H_VISIBLE) & (self.vcount < V_VISIBLE)),

            # HSYNC (active low during sync pulse)
            self.hsync.eq(~((self.hcount >= HS_BEG) & (self.hcount < HS_END))),
            # VSYNC (active low)
            self.vsync.eq(~((self.vcount >= VS_BEG) & (self.vcount < VS_END))),
        ]

# ------------------- Sys-domain line engine -------------------
class LineEngineSRAM(LiteXModule):
    """Fill + read a line from tiny Wishbone.SRAM (double buffer). Pulses line_done at end of read."""
    def __init__(self, sram_base_words, fifo_sink):
        self.bus      = wishbone.Interface(data_width=32, adr_width=30)  # WB master
        self.req_y    = Signal(16)    # vcount_next (CDC from HDMI)
        self.req_tgl  = Signal()      # toggle (CDC)
        self.line_done= Signal()      # one sys-clock pulse when a row finishes PUSH_HI

        self.buf_sel  = Signal()
        self._tgl_d   = Signal()
        self.word_ix  = Signal(max=LINE_WORDS32)
        self.started  = Signal(reset=0)

        # y -> bar color (thresholds; 8 bars)
        color_idx = Signal(3)
        color16   = Signal(16)
        self.comb += [
            If(self.req_y >= 420, color_idx.eq(7)).Elif(
            self.req_y >= 360, color_idx.eq(6)).Elif(
            self.req_y >= 300, color_idx.eq(5)).Elif(
            self.req_y >= 240, color_idx.eq(4)).Elif(
            self.req_y >= 180, color_idx.eq(3)).Elif(
            self.req_y >= 120, color_idx.eq(2)).Elif(
            self.req_y >=  60, color_idx.eq(1)).Else(
                               color_idx.eq(0))
        ]
        self.comb += Case(color_idx, {
            0: color16.eq(0xF800),  # Red
            1: color16.eq(0x07E0),  # Green
            2: color16.eq(0x001F),  # Blue
            3: color16.eq(0xFFE0),  # Yellow
            4: color16.eq(0xF81F),  # Magenta
            5: color16.eq(0x07FF),  # Cyan
            6: color16.eq(0xFFFF),  # White
            7: color16.eq(0x0000),  # Black
        })

        # double-buffer bases (word addressing)
        buf0_base = Signal(len(self.bus.adr))
        buf1_base = Signal(len(self.bus.adr))
        self.comb += [
            buf0_base.eq(sram_base_words),
            buf1_base.eq(sram_base_words + LINE_WORDS32)
        ]
        cur_base = Signal.like(buf0_base)
        self.comb += cur_base.eq(Mux(self.buf_sel, buf1_base, buf0_base))

        # WB helpers
        def wb_write(addr, data):
            return [ self.bus.cyc.eq(1), self.bus.stb.eq(1), self.bus.we.eq(1),
                     self.bus.sel.eq(0xF), self.bus.adr.eq(addr), self.bus.dat_w.eq(data) ]
        def wb_read(addr):
            return [ self.bus.cyc.eq(1), self.bus.stb.eq(1), self.bus.we.eq(0),
                     self.bus.sel.eq(0xF), self.bus.adr.eq(addr) ]

        fsm = FSM(reset_state="IDLE"); self.submodules += fsm

        new_req = Signal()
        self.sync += self._tgl_d.eq(self.req_tgl)
        self.comb += new_req.eq(self.req_tgl ^ self._tgl_d)

        wdata = Signal(32)
        self.comb += wdata.eq(Cat(color16, color16))

        # default for line_done
        self.sync += self.line_done.eq(0)

        fsm.act("IDLE",
            If(~self.started,
                NextValue(self.started, 1),
                NextValue(self.word_ix, 0),
                NextState("FILL")
            ).Elif(new_req,
                NextValue(self.word_ix, 0),
                NextState("FILL")
            )
        )
        fsm.act("FILL",
            *wb_write(cur_base + self.word_ix, wdata),
            If(self.bus.ack,
                If(self.word_ix == (LINE_WORDS32 - 1),
                    NextValue(self.word_ix, 0),
                    NextState("READ")
                ).Else(
                    NextValue(self.word_ix, self.word_ix + 1)
                )
            )
        )

        cur_word = Signal(32)
        fsm.act("READ",
            *wb_read(cur_base + self.word_ix),
            If(self.bus.ack,
                NextValue(cur_word, self.bus.dat_r),
                NextState("PUSH_LO")
            )
        )
        fsm.act("PUSH_LO",
            fifo_sink.valid.eq(1),
            fifo_sink.data.eq(cur_word[0:16]),
            If(fifo_sink.ready, NextState("PUSH_HI"))
        )
        fsm.act("PUSH_HI",
            fifo_sink.valid.eq(1),
            fifo_sink.data.eq(cur_word[16:32]),
            If(fifo_sink.ready,
                If(self.word_ix == (LINE_WORDS32 - 1),
                    NextValue(self.buf_sel, ~self.buf_sel),
                    NextValue(self.line_done, 1),     # pulse when a row fully pushed
                    NextState("IDLE")
                ).Else(
                    NextValue(self.word_ix, self.word_ix + 1),
                    NextState("READ")
                )
            )
        )

# ------------------------------ SoC ----------------------------------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(48e6), **kwargs):
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0x8000)
        kwargs.setdefault("integrated_sram_size", 0x2000)
        kwargs.setdefault("with_uart", False)

        platform = sipeed_tang_nano_20k.Platform()

        try:
            SoCCore.__init__(self, platform,
                             clk_freq=sys_clk_freq,
                             ident="Tang Nano 20K — HDMI bars via single Wishbone.SRAM (simple timing)",
                             **kwargs)
        except TypeError:
            SoCCore.__init__(self, platform, sys_clk_freq,
                             ident="Tang Nano 20K — HDMI bars via single Wishbone.SRAM (simple timing)",
                             **kwargs)

        # Clocks
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Tiny Wishbone.SRAM (double line buffer) as a SLAVE region
        self.submodules.line_sram = wishbone.SRAM(
            mem_or_size = SRAM_BYTES,
            read_only   = False,
            bus         = wishbone.Interface(data_width=32, address_width=32, addressing="word")
        )
        self.bus.add_slave("linebuf", self.line_sram.bus,
                           SoCRegion(origin=0x50000000, size=0x1000, cached=True))
        sram_base_words = (0x50000000 >> 2)

        # Simple HDMI timing (HDMI domain)
        self.submodules.tmg = SimpleTimingHDMI()

        # Async FIFO sys->hdmi for 16-bit pixels
        pix_fifo = AsyncFIFO([("data", 16)], depth=1024)
        pix_fifo = ClockDomainsRenamer({"write":"sys", "read":"hdmi"})(pix_fifo)
        self.submodules.pix_fifo = pix_fifo

        # Sys line engine (WB master)
        self.submodules.lineeng = LineEngineSRAM(
            sram_base_words = sram_base_words,
            fifo_sink       = self.pix_fifo.sink
        )
        try:
            self.bus.add_master(name="lineeng", master=self.lineeng.bus)
        except AttributeError:
            self.add_wb_master(self.lineeng.bus)

        # Prefetch handshake: on each DE rising, request next line number
        vcount_next_hdmi = Signal(16)
        de_d = Signal()
        req_tog_hdmi = Signal(reset=0)
        self.sync.hdmi += de_d.eq(self.tmg.de)
        new_line = Signal(); self.comb += new_line.eq(~de_d & self.tmg.de)

        self.sync.hdmi += [
            If(new_line,
                vcount_next_hdmi.eq(Mux(self.tmg.vcount == (V_VISIBLE-1), 0, self.tmg.vcount + 1)),
                req_tog_hdmi.eq(~req_tog_hdmi)
            )
        ]
        vcount_next_sys = Signal(16)
        req_tog_sys     = Signal()
        self.specials += MultiReg(vcount_next_hdmi, vcount_next_sys, "sys")
        self.specials += MultiReg(req_tog_hdmi,   req_tog_sys,     "sys")
        self.comb += [
            self.lineeng.req_y.eq(vcount_next_sys),
            self.lineeng.req_tgl.eq(req_tog_sys),
        ]

        # HDMI PHY
        pads = platform.request("hdmi")
        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(
            VideoGowinHDMIPHY(pads)
        )

        # Pixel path
        pix16 = Signal(16)
        r8,g8,b8 = rgb565_to_888(pix16)

        pop = Signal()
        self.comb += [
            pop.eq(self.tmg.de & self.pix_fifo.source.valid),  # always pop on DE if data present
            self.pix_fifo.source.ready.eq(pop),
        ]

        # Underflow (sticky per frame)
        underflow = Signal(reset=0)
        self.sync.hdmi += [
            If(self.tmg.de & ~self.pix_fifo.source.valid, underflow.eq(1)),
            # clear at frame blank (vsync trailing edge or when not valid)
            If(~self.tmg.valid, underflow.eq(0))
        ]

        self.sync.hdmi += [
            If(pop,
                pix16.eq(self.pix_fifo.source.data)
            ).Elif(~self.tmg.de,
                pix16.eq(0)
            )
        ]

        self.comb += [
            self.videophy.sink.valid.eq(self.tmg.valid),
            self.videophy.sink.de.eq(self.tmg.de),
            self.videophy.sink.hsync.eq(self.tmg.hsync),
            self.videophy.sink.vsync.eq(self.tmg.vsync),
            self.videophy.sink.r.eq(r8),
            self.videophy.sink.g.eq(g8),
            self.videophy.sink.b.eq(b8),
        ]

        # ----------------- LEDs -----------------
        leds = platform.request_all("led_n")
        # LED0 sys heartbeat, LED1 hdmi heartbeat
        hb_sys  = Signal(26); self.sync     += hb_sys.eq(hb_sys + 1)
        hb_hdmi = Signal(24); self.sync.hdmi+= hb_hdmi.eq(hb_hdmi + 1)
        # LED3: DE edge detector (slow counter in HDMI domain)
        de_edge_cnt = Signal(16); self.sync.hdmi += If(new_line, de_edge_cnt.eq(de_edge_cnt + 1))
        # LED4: row finished pushing (slow counter in SYS domain)
        line_done_cnt = Signal(16); self.sync += If(self.lineeng.line_done, line_done_cnt.eq(line_done_cnt + 1))

        assigns = []
        if len(leds) >= 1: assigns.append(leds[0].eq(~hb_sys[25]))
        if len(leds) >= 2: assigns.append(leds[1].eq(~hb_hdmi[23]))
        if len(leds) >= 3: assigns.append(leds[2].eq(~(~underflow)))     # ON=OK, OFF=underflow
        if len(leds) >= 4: assigns.append(leds[3].eq(~de_edge_cnt[12]))  # ~7 Hz if DE edges happen
        if len(leds) >= 5: assigns.append(leds[4].eq(~line_done_cnt[12]))
        self.comb += assigns

# --------------------------- Build/Load ------------------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform,
        description="Tang Nano 20K — HDMI bars via single Wishbone.SRAM (simple timing) with LED diagnostics")
    p.add_target_argument("--sys-clk-freq", default=48e6, type=float)
    args = p.parse_args()

    soc = Top(sys_clk_freq=int(args.sys_clk_freq))
    bld = Builder(soc)
    bld.build()
    prog = soc.platform.create_programmer()
    try:
        bit = bld.get_bitstream_filename(mode="sram")
    except TypeError:
        bit = bld.get_bitstream_filename()
    prog.load_bitstream(bit)

if __name__ == "__main__":
    main()
