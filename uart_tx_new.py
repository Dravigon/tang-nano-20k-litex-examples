from migen import *
from litex.gen import *
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.uart import UARTPHY, UART
from litex.soc.integration.soc import SoCRegion
from litex.build.io import DDROutput
from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY

from uart_hello import UartHello

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()
        clk27 = platform.request("clk27")
        pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.pll = pll

from litex.soc.cores.uart import UART
from uart_hello import UartHello   # the tiny helper that feeds the sink

class MySoC(SoCCore):
    def __init__(self, sys_clk_freq=int(27e6)):
        p = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(p, sys_clk_freq)

        SoCCore.__init__(self, p, sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0, integrated_main_ram_size=0,
                         with_uart=False)

        pads = p.request("custom_serial")

        # ✅ Direct PHY, no wrapper
        self.submodules.uartphy = UARTPHY(pads, sys_clk_freq, baudrate=115200)

        # ✅ Drive the PHY's sink directly
        self.submodules.hello = UartHello(self.uartphy.sink,
                                          text="Hello, custom UART!\r\n",
                                          start_hold_cycles=2048)

        # heartbeat
        led = p.request("led_n", 0)
        cnt = Signal(25); self.sync += cnt.eq(cnt + 1)
        self.comb += led.eq(~cnt[24])



def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--sys-clk-freq", default=27e6, type=float)
    args = parser.parse_args()

    soc = MySoC( sys_clk_freq=args.sys_clk_freq)
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
