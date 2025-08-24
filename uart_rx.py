# soc_uart_rx.py
from migen import *
from litex.gen import *
from platforms import sipeed_tang_nano_20k

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.uart import UARTPHY

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.pll = pll

class MySoC(SoCCore):
    def __init__(self, sys_clk_freq=int(27e6)):
        p = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = CRG(p, sys_clk_freq)

        SoCCore.__init__(self, p, sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0, integrated_main_ram_size=0,
                         with_uart=False)   # we use UARTPHY directly

        # --- UART PHY on your custom pins (RX/TX) ---
        pads = p.request("custom_serial")
        self.submodules.uartphy = UARTPHY(pads, sys_clk_freq, baudrate=115200)

        # after: self.submodules.uartphy = UARTPHY(pads, sys_clk_freq, baudrate=115200)
        rx = self.uartphy.source   # bytes from ESP32
        tx = self.uartphy.sink     # bytes to ESP32
        
        pending = Signal(reset=0)
        tx_data = Signal(8)
        
        # default
        self.sync += tx.valid.eq(0)
        
        self.sync += [
            # capture a new byte if none pending
            If(~pending & rx.valid,
                tx_data.eq(rx.data),
                pending.eq(1)
            ),
        
            # if we have something pending, present it until accepted
            If(pending,
                tx.valid.eq(1),
                tx.data.eq(tx_data),
                If(tx.ready,
                    pending.eq(0)    # byte consumed; drop valid next cycle
                )
            )
        ]


        # --- Show lower 6 bits of last_byte on LEDs (active-low) ---
        for i in range(6):
            led_i = p.request("led_n", i)
            self.comb += led_i.eq(~tx_data[i])

        # --- Heartbeat so you know bitstream is alive ---
        hb = Signal(25)
        self.sync += hb.eq(hb + 1)
        # If you want a dedicated heartbeat LED (e.g., led_n[5]), OR it with a bit:
        # self.comb += p.request("led_n", 5).eq(~(last_byte[5] | hb[24]))

def main():
    from litex.build.parser import LiteXArgumentParser
    ap = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    # ap.add_target_argument("--build", action="store_true")
    # ap.add_target_argument("--load",  action="store_true")
    args = ap.parse_args()

    soc = MySoC()
    bld = Builder(soc)
    if args.build: bld.build()
    if args.load:  soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
