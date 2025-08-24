from migen import *
from litex.gen import *

from litex_boards.platforms import sipeed_tang_nano_20k
from litex_boards.targets.sipeed_tang_nano_20k import _CRG as CRG

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

class BlinkSoC(SoCCore):
    def __init__(self, **kwargs):
        
        platform = sipeed_tang_nano_20k.Platform()
        clk_freq = int(27e6)

        SoCCore.__init__(self, platform, clk_freq=clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0,
                         integrated_sram_size=0x1000,
                         with_uart=False)

        # ✅ Fix: pass platform, not clk27
        self.submodules.crg = CRG(platform, clk_freq)

        led = platform.request("led_n", 0)  # use one of the defined LED names
        counter = Signal(24)
        self.sync += counter.eq(counter + 1)
        self.comb += led.eq(~counter[23])  # invert because it's active-low




# Build --------------------------------------------------------------------------------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform, description="Tiny LiteX SoC on Tang Nano 20K.")
    parser.add_target_argument("--flash",                action="store_true",      help="Flash Bitstream.")
    args = parser.parse_args()
    soc = BlinkSoC( **parser.soc_argdict)
    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        prog = soc.platform.create_programmer("openfpgaloader")
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))
    if args.flash:
        prog = soc.platform.create_programmer("openfpgaloader")
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".fs")) 

if __name__ == "__main__":
    main()
