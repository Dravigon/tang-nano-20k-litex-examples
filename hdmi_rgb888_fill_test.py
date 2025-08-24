#!/usr/bin/env python3
# Tang Nano 20K — HDMI gradient smoke test (no SDRAM)
# VTG -> GradientRenderer -> VideoGowinHDMIPHY (640x480@60)

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.soc.interconnect import stream
from litex_boards.platforms import sipeed_tang_nano_20k

# --- Stream layouts used by LiteX video cores ---
video_timing_layout = [("hsync",1), ("vsync",1), ("de",1)]
video_data_layout   = [("hsync",1), ("vsync",1), ("de",1), ("r",8), ("g",8), ("b",8)]

HRES, VRES = 640, 480

# -----------------------------------------------------------------------------
# CRG: 27 MHz in -> SYS, HDMI (25.175 MHz), HDMI5x (125.875 MHz via /5)
# -----------------------------------------------------------------------------
class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(50e6)):
        self.cd_sys    = ClockDomain()
        self.cd_por    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")

        # POR on 27 MHz
        por_cnt  = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(clk27), por_done.eq(por_cnt == 0)
        self.sync.por += If(~por_done, por_cnt.eq(por_cnt - 1))

        # SYS PLL
        self.submodules.sys_pll = sys_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += sys_pll.reset.eq(~por_done)
        sys_pll.register_clkin(clk27, 27e6)
        sys_pll.create_clkout(self.cd_sys, sys_clk_freq)

        # VIDEO PLL: make 125.875 MHz and divide by 5 -> ~25.175 MHz
        self.submodules.vid_pll = vid_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vid_pll.register_clkin(clk27, 27e6)
        vid_pll.create_clkout(self.cd_hdmi5x, 125_875_000, margin=1e-2)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5",
            i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk, o_CLKOUT=self.cd_hdmi.clk
        )

        # Hold resets low until POR is done
        # self.specials += [
        #     AsyncResetSynchronizer(self.cd_sys,    ~por_done),
        #     AsyncResetSynchronizer(self.cd_hdmi,   ~por_done),
        #     AsyncResetSynchronizer(self.cd_hdmi5x, ~por_done),
        # ]

# -----------------------------------------------------------------------------
# Gradient pattern generator (RGB888)
# -----------------------------------------------------------------------------
class GradientRenderer(LiteXModule):
    """R = x>>2, G = y>>1, B = (x^y)>>2 within active video."""
    def __init__(self, hres=640, vres=480):
        self.vtg_sink = stream.Endpoint(video_timing_layout)  # timing in
        self.source   = stream.Endpoint(video_data_layout)    # rgb out

        x = Signal(11)  # enough for 640
        y = Signal(11)  # enough for 480

        # Count pixels only when in DE; advance y at end of each active line
        self.sync += [
            If(self.vtg_sink.valid & self.source.ready,
                If(self.vtg_sink.de,
                    If(x == (hres-1),
                        x.eq(0),
                        If(y == (vres-1), y.eq(0)).Else(y.eq(y + 1))
                    ).Else(
                        x.eq(x + 1)
                    )
                ).Else(
                    x.eq(0)
                )
            )
        ]

        r8 = Signal(8); g8 = Signal(8); b8 = Signal(8)
        self.comb += [
            r8.eq(x[2:10]),           # x >> 2
            g8.eq(y[1:9]),            # y >> 1
            b8.eq((x ^ y)[2:10]),     # (x^y) >> 2

            # Pass timing through + gate RGB by DE
            self.source.valid.eq(self.vtg_sink.valid),
            self.vtg_sink.ready.eq(self.source.ready),
            self.source.hsync.eq(self.vtg_sink.hsync),
            self.source.vsync.eq(self.vtg_sink.vsync),
            self.source.de.eq(self.vtg_sink.de),
            self.source.r.eq(Mux(self.vtg_sink.de, r8, 0)),
            self.source.g.eq(Mux(self.vtg_sink.de, g8, 0)),
            self.source.b.eq(Mux(self.vtg_sink.de, b8, 0)),
        ]

# -----------------------------------------------------------------------------
# SoC Top
# -----------------------------------------------------------------------------
class Top(SoCCore):
    def __init__(self):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = CRG(platform)

        SoCCore.__init__(self, platform, int(50e6),
            cpu_type=None,
            integrated_rom_size=0x4000,
            integrated_main_ram_size=0x2000,  # tiny; no external RAM
            with_uart=False
        )

        # HDMI pads; force HPD high so PHY doesn't mute TMDS during bring-up
        pads = platform.request("hdmi")
        class PadsWrap:
            def __init__(self, p):
                for n in ["clk_p","clk_n","data0_p","data0_n","data1_p","data1_n","data2_p","data2_n","sda","scl","cec"]:
                    if hasattr(p, n): setattr(self, n, getattr(p, n))
                self.hdp = 1
        pads_w = PadsWrap(pads)

        # HDMI PHY lives in 'sys'/'sys5x' internally → map to 'hdmi'/'hdmi5x'
        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(
            VideoGowinHDMIPHY(pads_w)
        )

        # VTG + Gradient in the hdmi pixel domain
        self.submodules.vtg = ClockDomainsRenamer("hdmi")(
            VideoTimingGenerator(default_video_timings="640x480@60Hz")
        )
        self.submodules.grad = ClockDomainsRenamer("hdmi")(GradientRenderer(HRES, VRES))

        # Wire streams: VTG -> Gradient -> PHY
        self.comb += [
            self.vtg.source.connect(self.grad.vtg_sink,  keep={"valid","ready","de","hsync","vsync"}),
            self.grad.source.connect(self.videophy.sink),
        ]

        # Debug LEDs: sys + pixel heartbeats (active-low LEDs)
        hb_sys = Signal(26); self.sync += hb_sys.eq(hb_sys + 1)
        self.comb += platform.request("led_n", 5).eq(~hb_sys[25])

        hb_pix = Signal(24); self.sync.hdmi += hb_pix.eq(hb_pix + 1)
        self.comb += platform.request("led_n", 4).eq(~hb_pix[23])

def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = p.parse_args()
    soc  = Top()
    bld  = Builder(soc)
    if args.build: bld.build()
    if args.load:  soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
