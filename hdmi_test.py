#!/usr/bin/env python3
from migen import *
from migen.genlib.cdc import ClockDomainsRenamer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.soc.interconnect import stream
from litex.build.generic_platform import ConstraintError
from platforms import sipeed_tang_nano_20k  # your local platform

video_timing_layout = [("hsync",1), ("vsync",1), ("de",1)]
video_data_layout   = [("hsync",1), ("vsync",1), ("de",1), ("r",8), ("g",8), ("b",8)]

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(50e6)):
        self.cd_sys    = ClockDomain()
        self.cd_por    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")

        # POR
        por_cnt  = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(clk27), por_done.eq(por_cnt == 0)
        self.sync.por += If(~por_done, por_cnt.eq(por_cnt - 1))

        # SYS PLL
        self.submodules.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # VIDEO PLL: 25.175 MHz pixel (125.875 MHz *1/5)
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, 125_875_000, margin=1e-2)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5",
            i_RESETN=1,
            i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk,
            o_CLKOUT=self.cd_hdmi.clk
        )

class ColorBars(LiteXModule):
    def __init__(self, hres=640, vres=480):
        self.vtg_sink = stream.Endpoint(video_timing_layout)
        self.source   = stream.Endpoint(video_data_layout)

        x = Signal(11); y = Signal(11)
        self.sync.hdmi += [
            If(self.vtg_sink.valid & self.source.ready,
                If(self.vtg_sink.de,
                    If(x == (hres-1),
                        x.eq(0),
                        If(y == (vres-1), y.eq(0)).Else(y.eq(y+1))
                    ).Else(x.eq(x + 1))
                ).Else(x.eq(0))
            )
        ]
        bar = Signal(3)
        self.comb += bar.eq(x[6:9])  # 8 bars
        r=g=b=Signal(8)
        r8=g8=b8=Signal(8)  # helpers
        self.comb += [
            Case(bar, {
                0:[r8.eq(255), g8.eq(0),   b8.eq(0)],
                1:[r8.eq(255), g8.eq(255), b8.eq(0)],
                2:[r8.eq(0),   g8.eq(255), b8.eq(0)],
                3:[r8.eq(0),   g8.eq(255), b8.eq(255)],
                4:[r8.eq(0),   g8.eq(0),   b8.eq(255)],
                5:[r8.eq(255), g8.eq(0),   b8.eq(255)],
                6:[r8.eq(255), g8.eq(255), b8.eq(255)],
                7:[r8.eq(32),  g8.eq(32),  b8.eq(32)],
            }),
            self.source.valid.eq(self.vtg_sink.valid),
            self.vtg_sink.ready.eq(self.source.ready),
            self.source.hsync.eq(self.vtg_sink.hsync),
            self.source.vsync.eq(self.vtg_sink.vsync),
            self.source.de.eq(self.vtg_sink.de),
            self.source.r.eq(Mux(self.vtg_sink.de, r8, 0)),
            self.source.g.eq(Mux(self.vtg_sink.de, g8, 0)),
            self.source.b.eq(Mux(self.vtg_sink.de, b8, 0)),
        ]

class HDMI_Smoke(SoCCore):
    def __init__(self):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = CRG(platform)

        SoCCore.__init__(self, platform, int(50e6),
            cpu_type=None,
            integrated_rom_size=0x4000,
            integrated_main_ram_size=0x2000,
            with_uart=False
        )

        pads = platform.request("hdmi")

        # Tie/observe HPD: LED3 shows HPD, and force it high into the PHY to avoid gating
        hpd = Signal(reset=1)
        if hasattr(pads, "hdp"):
            hpd_in = pads.hdp
            self.comb += platform.request("led_n", 3).eq(~hpd_in)  # LED3 on when HPD=1
        # Wrapper that gives the PHY a high HPD regardless of connector
        class PadsWrap:
            def __init__(self, p, force_hpd):
                for n in ["clk_p","clk_n","data0_p","data0_n","data1_p","data1_n","data2_p","data2_n","sda","scl","cec"]:
                    if hasattr(p, n): setattr(self, n, getattr(p, n))
                self.hdp = force_hpd
        pads_w = PadsWrap(pads, 1)

        self.submodules.videophy = VideoGowinHDMIPHY(
            pads_w, clock_domain="hdmi",
        )

        self.submodules.vtg  = ClockDomainsRenamer("hdmi")(
            VideoTimingGenerator(default_video_timings="640x480@60Hz")
        )
        self.submodules.bars = ClockDomainsRenamer("hdmi")(ColorBars(640, 480))
        self.comb += [
            self.vtg.source.connect(self.bars.vtg_sink, keep={"valid","ready","de","hsync","vsync"}),
            self.bars.source.connect(self.videophy.sink),
        ]

        # DEBUG LEDs:
        # LED5: sys heartbeat
        hb_sys = Signal(26); self.sync += hb_sys.eq(hb_sys + 1)
        self.comb += platform.request("led_n", 5).eq(~hb_sys[25])
        # LED4: hdmi pixel-domain heartbeat
        hb_pix = Signal(24); self.sync.hdmi += hb_pix.eq(hb_pix + 1)
        self.comb += platform.request("led_n", 4).eq(~hb_pix[23])

def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = p.parse_args()
    soc = HDMI_Smoke()
    bld = Builder(soc)
    if args.build: bld.build()
    if args.load:  soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))
if __name__ == "__main__":
    main()
