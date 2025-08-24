# uart_rx_color_hdmi.py
from migen import *
from migen.genlib.cdc import ClockDomainsRenamer
from litex.gen import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.uart import UARTPHY
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.soc.interconnect import stream

from platforms import sipeed_tang_nano_20k

# --- Simple stream layouts ---
video_timing_layout = [("hsync",1), ("vsync",1), ("de",1)]
video_data_layout   = [("hsync",1), ("vsync",1), ("de",1), ("r",8), ("g",8), ("b",8)]

# --- CRG: single source of truth for resets/clocks ---
from migen.genlib.resetsync import AsyncResetSynchronizer
class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_hdmi=True):
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



# --- UART packet parser: 0xAA 0x55 R G B, ACK 'K' ---
class RGBFromUART(LiteXModule):
    def __init__(self, phy):
        rx, tx = phy.source, phy.sink
        self.r = Signal(8)
        self.g = Signal(8)
        self.b = Signal(8)

        got1, got2 = Signal(reset=0), Signal(reset=0)
        idx        = Signal(2, reset=0)
        pending    = Signal(reset=0)
        ack        = Signal(8, reset=ord('K'))

        # default: valid low unless sending
        self.sync += tx.valid.eq(0)

        self.sync += [
            If(~got1,
                If(rx.valid & (rx.data == 0xAA), got1.eq(1))
            ).Elif(~got2,
                If(rx.valid,
                    If(rx.data == 0x55, got2.eq(1)).Else(got1.eq(0))
                )
            ).Else(
                If(rx.valid,
                    Case(idx, {
                        0: [ self.r.eq(rx.data), idx.eq(1) ],
                        1: [ self.g.eq(rx.data), idx.eq(2) ],
                        2: [
                            self.b.eq(rx.data),
                            idx.eq(0), got1.eq(0), got2.eq(0),
                            pending.eq(1)
                        ]
                    })
                )
            ),
            If(pending,
                tx.valid.eq(1),
                tx.data.eq(ack),
                If(tx.ready, pending.eq(0))
            )
        ]

# --- Solid-color generator (VTG -> RGB data) ---
class SolidColor(Module):
    def __init__(self):
        self.vtg_sink = stream.Endpoint(video_timing_layout)   # timing in
        self.source   = stream.Endpoint(video_data_layout)     # rgb out
        self.r, self.g, self.b = Signal(8), Signal(8), Signal(8)

        self.comb += [
            self.source.valid.eq(self.vtg_sink.valid),
            self.vtg_sink.ready.eq(self.source.ready),
            self.source.hsync.eq(self.vtg_sink.hsync),
            self.source.vsync.eq(self.vtg_sink.vsync),
            self.source.de.eq(self.vtg_sink.de),
            self.source.r.eq(Mux(self.vtg_sink.de, self.r, 0)),
            self.source.g.eq(Mux(self.vtg_sink.de, self.g, 0)),
            self.source.b.eq(Mux(self.vtg_sink.de, self.b, 0)),
        ]

# --- Top SoC ---
class MySoC(SoCCore):
    def __init__(self, sys_clk_freq=int(27e6)):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = CRG(platform, sys_clk_freq)

        SoCCore.__init__(self, platform, sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0x4000,   # simple, no external SDRAM
                         integrated_main_ram_size=0x4000,
                         with_uart=False)

        # UART (custom pins)
        pads = platform.request("custom_serial")
        self.submodules.uartphy = UARTPHY(pads, sys_clk_freq, baudrate=115200)
        self.submodules.rgbctl  = RGBFromUART(self.uartphy)

        # LEDs show brightness (active-low)
        for i, sig in enumerate([self.rgbctl.r, self.rgbctl.g, self.rgbctl.b]):
            led = platform.request("led_n", i)
            pwm = Signal(8)
            self.sync += pwm.eq(pwm + 1)
            self.comb += led.eq(~(pwm < sig))

        # Video: VTG + solid color in pixel domain, TMDS PHY in hdmi/hdmi5x
        self.submodules.vtg = ClockDomainsRenamer("hdmi")(
            VideoTimingGenerator(default_video_timings="640x480@60Hz")
        )
        self.submodules.color = ClockDomainsRenamer("hdmi")(SolidColor())
        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi", "sys5x":"hdmi5x"})(
            VideoGowinHDMIPHY(platform.request("hdmi"))
        )

        # Color from UART
        self.comb += [
            self.color.r.eq(self.rgbctl.r),
            self.color.g.eq(self.rgbctl.g),
            self.color.b.eq(self.rgbctl.b),
        ]

        # Stream wiring
        self.comb += [
            self.vtg.source.connect(self.color.vtg_sink, keep={"valid","ready","de","hsync","vsync"}),
            self.color.source.connect(self.videophy.sink),
        ]

        # Heartbeat (LED 5)
        hb = Signal(25)
        self.sync += hb.eq(hb + 1)
        self.comb += platform.request("led_n", 5).eq(~hb[24])

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    # parser.add_target_argument("--build", action="store_true")
    # parser.add_target_argument("--load",  action="store_true")
    args = parser.parse_args()

    soc = MySoC()
    bld = Builder(soc)
    if args.build:
        bld.build()
    if args.load:
        soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
