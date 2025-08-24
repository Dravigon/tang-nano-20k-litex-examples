#!/usr/bin/env python3
# Tang Nano 20K — Minimal HDMI color bars (no SDRAM, no FIFO)
# - VTG runs in "hdmi" domain
# - PHY: VideoGowinHDMIPHY fed directly with a test pattern
#
# Build:
#   python3 hdmi_min_bars.py --build
# Load:
#   python3 hdmi_min_bars.py --load

from migen import *
from migen.genlib.cdc import MultiReg, ClockDomainsRenamer

from litex.gen import LiteXModule
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY

# Your platform module (make sure HDMI is defined once)
from platforms import sipeed_tang_nano_20k

# ----------------------------- CRG -----------------------------------
PIXCLK  = int(25_200_000)   # 640x480@60
PIX5X   = PIXCLK*5

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

        # Video PLL: 5x pixel clock
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIX5X, margin=1e-2)

        # Divide by 5 to pixel clock
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5",
            i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk,
            o_CLKOUT=self.cd_hdmi.clk
        )

# -------------------------- Test Pattern -----------------------------
def bars_rgb(v):
    """Return (r,g,b) tuples by vertical ranges (8 bars of 60 lines each)."""
    r = Signal(8); g = Signal(8); b = Signal(8)

    # If/Elif chain is the most reliable in pure Migen
    cases = [
        (0,   60,  (0xFF, 0x00, 0x00)),  # Red
        (60,  120, (0x00, 0xFF, 0x00)),  # Green
        (120, 180, (0x00, 0x00, 0xFF)),  # Blue
        (180, 240, (0xFF, 0xFF, 0x00)),  # Yellow
        (240, 300, (0xFF, 0x00, 0xFF)),  # Magenta
        (300, 360, (0x00, 0xFF, 0xFF)),  # Cyan
        (360, 420, (0xFF, 0xFF, 0xFF)),  # White
        (420, 480, (0x00, 0x00, 0x00)),  # Black
    ]

    # Build the combinatorial assignments
    stmt = []
    for y0, y1, (R, G, B) in cases:
        stmt.append( If((v >= y0) & (v < y1), r.eq(R), g.eq(G), b.eq(B)) )
    return r, g, b, stmt

# ------------------------------ SoC ----------------------------------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(48e6), **kwargs):
        # No CPU/BIOS needed — just enough SRAM for LiteX CSR if required
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0)
        kwargs.setdefault("integrated_sram_size", 0x2000)
        kwargs.setdefault("with_uart", False)

        platform = sipeed_tang_nano_20k.Platform()

        try:
            SoCCore.__init__(self, platform,
                             clk_freq=sys_clk_freq,
                             ident="Tang Nano 20K — Minimal HDMI color bars",
                             **kwargs)
        except TypeError:
            SoCCore.__init__(self, platform, sys_clk_freq,
                             ident="Tang Nano 20K — Minimal HDMI color bars",
                             **kwargs)

        # Clocks / domains
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Video timing in HDMI clock domain
        self.submodules.vtg = ClockDomainsRenamer("hdmi")(
            VideoTimingGenerator(default_video_timings="640x480@60Hz")
        )

        # HDMI PHY in HDMI clock domain
        hdmi_pads = platform.request("hdmi")
        self.submodules.hdmi_phy = VideoGowinHDMIPHY(hdmi_pads, clock_domain="hdmi")

        # Latch coordinates during DE so colors are stable
        h = Signal.like(self.vtg.source.hcount)
        v = Signal.like(self.vtg.source.vcount)
        de = self.vtg.source.de

        self.sync.hdmi += [
            If(de,
                h.eq(self.vtg.source.hcount),
                v.eq(self.vtg.source.vcount)
            )
        ]

        # Bars
        r,g,b,stmts = bars_rgb(v)
        self.comb += [
            If(de, *stmts).Else([r.eq(0), g.eq(0), b.eq(0)])
        ]

        # Drive PHY
        self.comb += [
            self.hdmi_phy.sink.valid.eq(self.vtg.source.valid),
            self.hdmi_phy.sink.de.eq(self.vtg.source.de),
            self.hdmi_phy.sink.hsync.eq(self.vtg.source.hsync),
            self.hdmi_phy.sink.vsync.eq(self.vtg.source.vsync),
            self.hdmi_phy.sink.r.eq(r),
            self.hdmi_phy.sink.g.eq(g),
            self.hdmi_phy.sink.b.eq(b),
        ]

        # ----------------- LEDs for quick debug -----------------
        leds = platform.request_all("led_n")
        hb_sys  = Signal(26); self.sync     += hb_sys.eq(hb_sys + 1)
        hb_hdmi = Signal(24); self.sync.hdmi+= hb_hdmi.eq(hb_hdmi + 1)

        # Activity toggles in HDMI domain
        vs_tgl  = Signal(); de_tgl  = Signal(); rdy_tgl = Signal()
        self.sync.hdmi += [
            vs_tgl.eq(vs_tgl ^ self.vtg.source.vsync),
            de_tgl.eq(de_tgl ^ self.vtg.source.de),
            rdy_tgl.eq(rdy_tgl ^ self.hdmi_phy.sink.ready),
        ]

        # Map: LED0=sys HB, LED1=hdmi HB, LED2=VSYNC, LED3=DE, LED4=PHY ready
        assigns = []
        if len(leds) >= 1: assigns += [leds[0].eq(~hb_sys[25])]
        if len(leds) >= 2: assigns += [leds[1].eq(~hb_hdmi[23])]
        if len(leds) >= 3: assigns += [leds[2].eq(~vs_tgl)]
        if len(leds) >= 4: assigns += [leds[3].eq(~de_tgl)]
        if len(leds) >= 5: assigns += [leds[4].eq(~rdy_tgl)]
        if assigns: self.comb += assigns

# --------------------------- Build/Load ------------------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform,
        description="Tang Nano 20K — Minimal HDMI color bars (no SDRAM)")
    p.add_target_argument("--sys-clk-freq", default=48e6, type=float)
    args = p.parse_args()

    soc = Top(sys_clk_freq=int(args.sys_clk_freq))
    bld = Builder(soc)

    if args.build:
        bld.build()
    if args.load:
        prog = soc.platform.create_programmer()
        try:
            bit = bld.get_bitstream_filename(mode="sram")
        except TypeError:
            bit = bld.get_bitstream_filename()
        prog.load_bitstream(bit)

if __name__ == "__main__":
    main()
