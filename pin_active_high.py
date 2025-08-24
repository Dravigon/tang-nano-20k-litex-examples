# force_high_test.py
from migen import *
from litex.gen import *
from platforms import sipeed_tang_nano_20k
# from litex_boards.targets.sipeed_tang_nano_20k import _CRG as CRG

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

class _CRG(LiteXModule):
    def __init__(self, p, f=27e6):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(devicename=p.devicename, device=p.device)
        pll.register_clkin(p.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, f)
        self.pll = pll

# class CRG(LiteXModule):
#     def __init__(self, platform, sys_clk_freq):
#         self.cd_sys = ClockDomain()
#         clk27 = platform.request("clk27")
#         pll = GW2APLL(devicename=platform.devicename, device=platform.device)
#         pll.register_clkin(clk27, 27e6)
#         pll.create_clkout(self.cd_sys, sys_clk_freq)
#         self.pll = pll
# raw_uart_tx.py
from migen import *

# raw_uart_tx.py
from migen import *

class RawUartTx(Module):
    def __init__(self, tx, clk_freq=27_000_000, baud=115200, byte=0x55):
        div_val = int(round(clk_freq / baud))  # 27e6/115200 ≈ 234
        assert div_val > 2

        divider = Signal(max=div_val, reset=div_val - 1)
        tick    = Signal()   # 1 clock-cycle pulse at baud rate
        sending = Signal(reset=0)  # expose for debug
        line    = Signal(reset=1)  # idle high
        data    = Signal(8, reset=byte)
        bitcnt  = Signal(4, reset=0)

        # Baud-rate tick generator
        self.sync += [
            tick.eq(0),
            If(divider == 0,
                divider.eq(div_val - 1),
                tick.eq(1)
            ).Else(
                divider.eq(divider - 1)
            )
        ]

        # Simple transmitter
        self.sync += [
            If(~sending,
                sending.eq(1),
                bitcnt.eq(0),
                data.eq(byte),
                line.eq(0)                  # start bit
            ).Elif(tick,
                If(bitcnt < 8,
                    line.eq(data[0]),
                    data.eq(Cat(data[1:], 0)),
                    bitcnt.eq(bitcnt + 1)
                ).Elif(bitcnt == 8,
                    line.eq(1),             # stop bit
                    bitcnt.eq(bitcnt + 1)
                ).Else(
                    sending.eq(0)           # restart next frame
                )
            )
        ]

        self.comb += tx.eq(line)

        # --- DEBUG OUTPUTS (optional to route to pins/LEDs) ---
        self.tick = tick
        self.sending = sending
        self.line = line


class SoC(SoCCore):
    def __init__(self):
        p = sipeed_tang_nano_20k.Platform()
        SoCCore.__init__(self, p, 27e6, cpu_type=None,
                         integrated_rom_size=0, integrated_main_ram_size=0,
                         with_uart=False)
        # ✅ Fix: pass platform, not clk27
        self.submodules.crg = _CRG(p, 27e6)

        # pads = p.request("custom_serial")
        # # Drive constant HIGH
        # self.comb += pads.tx.eq(0)
        pads = p.request("custom_serial")
        self.submodules.txraw = RawUartTx(pads.tx, clk_freq=27_000_000, baud=115200)



        led = p.request("led_n", 0)  # use one of the defined LED names
        counter = Signal(24)
        self.sync += counter.eq(counter + 1)
        self.comb += led.eq(~counter[23])  # invert because it's active-low


def main():
    from litex.build.parser import LiteXArgumentParser
    ap = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    # ap.add_target_argument("--build", action="store_true")
    # ap.add_target_argument("--load", action="store_true")
    a = ap.parse_args()
    soc = SoC(); b = Builder(soc)
    if a.build: b.build()
    if a.load:  soc.platform.create_programmer().load_bitstream(b.get_bitstream_filename(mode="sram"))
if __name__ == "__main__":
    main()
