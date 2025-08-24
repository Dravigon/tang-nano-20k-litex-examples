from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.build.io import DDROutput
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.gpio import GPIOIn
from litex.soc.cores.led import LedChaser, WS2812
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.interconnect import stream

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY

from litex_boards.platforms import sipeed_tang_nano_20k

# Constants
hbits = 12
vbits = 12

video_timing_layout = [
    ("hsync", 1), ("vsync", 1), ("de", 1),
    ("hres", hbits), ("vres", vbits),
    ("hcount", hbits), ("vcount", vbits),
]

video_data_layout = [
    ("hsync", 1), ("vsync", 1), ("de", 1),
    ("r", 8), ("g", 8), ("b", 8),
]

class ColorBarsPattern(LiteXModule):
    def __init__(self,platform):
        self.enable = Signal(reset=1)
        self.vtg_sink = vtg_sink = stream.Endpoint(video_timing_layout)
        self.source = source = stream.Endpoint(video_data_layout)

        enable = Signal()
        self.specials += MultiReg(self.enable, enable)

        pix = Signal(hbits)
        bar = Signal(3)

        btn = platform.request("btn", 0)       # Button (active High)
        led = platform.request("led_n", 1)     # LED (active low)

        state = Signal(reset=0)
        counter = Signal(24)

        self.sync += counter.eq(counter + 1)

        self.sync += [
            vtg_sink.connect(source, keep={"valid", "ready", "last", "de", "hsync", "vsync"}),
            If(counter == 0,
                If(btn,       # Active High
                    [state.eq(~state),
                    bar.eq(bar + 1)]
                )
            )
        ]

        self.comb += led.eq(~state)

        color_bar = [
            [0xff, 0xff, 0xff], [0xff, 0xff, 0x00],
            [0x00, 0xff, 0xff], [0x00, 0xff, 0x00],
            [0xff, 0x00, 0xff], [0xff, 0x00, 0x00],
            [0x00, 0x00, 0xff], [0x00, 0x00, 0x00]
        ]
        cases = {}
        for i, (r, g, b) in enumerate(color_bar):
            cases[i] = [source.r.eq(r), source.g.eq(g), source.b.eq(b)]
        self.comb += Case(bar, cases)

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_hdmi=False):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain()
        if with_hdmi:
            self.cd_hdmi = ClockDomain()
            self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(clk27)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        self.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        if with_hdmi:
            self.video_pll = video_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
            video_pll.register_clkin(clk27, 27e6)
            video_pll.create_clkout(self.cd_hdmi5x, 125e6, margin=1e-2)
            self.specials += Instance("CLKDIV",
                p_DIV_MODE="5",
                i_RESETN=1,
                i_CALIB=0,
                i_HCLKIN=self.cd_hdmi5x.clk,
                o_CLKOUT=self.cd_hdmi.clk
            )

def add_video_colorbars(soc, phy, timings="800x600@60Hz", clock_domain="sys",platform={}):
    from litex.soc.cores.video import VideoTimingGenerator

    vtg = VideoTimingGenerator(default_video_timings=timings)
    vtg = ClockDomainsRenamer(clock_domain)(vtg)
    soc.add_module(name="video_vtg", module=vtg)

    colorbars = ClockDomainsRenamer(clock_domain)(ColorBarsPattern(platform))
    soc.add_module(name="video_colorbars", module=colorbars)

    soc.comb += [
        vtg.source.connect(colorbars.vtg_sink),
        colorbars.source.connect(phy.sink)
    ]

class BaseSoC(SoCCore):
    def __init__(self, toolchain="gowin", sys_clk_freq=48e6, with_video_colorbars=False, **kwargs):
        platform = sipeed_tang_nano_20k.Platform(toolchain=toolchain)
        self.crg = _CRG(platform, sys_clk_freq, with_hdmi=with_video_colorbars)

        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Tang Nano 20K", **kwargs)

        if not self.integrated_main_ram_size:
            class SDRAMPads:
                def __init__(self):
                    p = platform
                    self.clk = p.request("O_sdram_clk")
                    self.cke = p.request("O_sdram_cke")
                    self.cs_n = p.request("O_sdram_cs_n")
                    self.cas_n = p.request("O_sdram_cas_n")
                    self.ras_n = p.request("O_sdram_ras_n")
                    self.we_n = p.request("O_sdram_wen_n")
                    self.dm = p.request("O_sdram_dqm")
                    self.a = p.request("O_sdram_addr")
                    self.ba = p.request("O_sdram_ba")
                    self.dq = p.request("IO_sdram_dq")
            pads = SDRAMPads()
            self.specials += DDROutput(0, 1, pads.clk, ClockSignal("sys"))
            self.sdrphy = GENSDRPHY(pads, sys_clk_freq)
            self.add_sdram("sdram", phy=self.sdrphy, module=M12L64322A(sys_clk_freq, "1:1"), l2_cache_size=128)

        if with_video_colorbars:
            self.videophy = VideoGowinHDMIPHY(platform.request("hdmi"), clock_domain="hdmi")
            add_video_colorbars(self, phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi",platform=self.platform)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--with-video-colorbars", action="store_true")
    parser.add_target_argument("--sys-clk-freq", default=27e6, type=float, help="System clock frequency.")
    args = parser.parse_args()

    soc = BaseSoC(
        toolchain=args.toolchain,
        sys_clk_freq=args.sys_clk_freq,
        with_video_colorbars=args.with_video_colorbars,
        **parser.soc_argdict
    )

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
