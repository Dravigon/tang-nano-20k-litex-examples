#!/usr/bin/env python3
# Tang Nano 20K — MISO stuck high to test wiring/mapping.
# If ESP32 does NOT read all 0xFF bytes, MISO pin/wiring/mapping is wrong.

from migen import *
from litex.gen import LiteXModule
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(54e6)):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

class MISOHigh(LiteXModule):
    def __init__(self, pads):
        # Drive MISO pad high at all times.
        self.comb += pads.miso.eq(1)

class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(54e6)):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(platform, sys_clk_freq=sys_clk_freq)

        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         cpu_type=None, integrated_rom_size=0x0,
                         integrated_main_ram_size=0x1000)

        pads = platform.request("custom_spi", 0)  # (SCK=51, MOSI=54, MISO=56, CS_n=48 in your file)
        self.submodules.miso_high = MISOHigh(pads)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--with-video-colorbars", default=True,action="store_true")
    parser.add_target_argument("--sys-clk-freq", default=27e6, type=float, help="System clock frequency.")
    args = parser.parse_args()

    soc = Top(sys_clk_freq=int(float(args.sys_clk_freq)))


    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
