# fpga_rgb.py (paste into your SoC file)
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

class RGBFromUART(LiteXModule):
    def __init__(self, phy, leds, sys_clk_freq):
        """
        phy: UARTPHY instance (with .source for RX, .sink for TX)
        leds: tuple/list of 3 active-low LED pad Signals (R,G,B-ish output)
        """

        rx, tx = phy.source, phy.sink

        # ----- parser state -----
        color_r = Signal(8, reset=0)
        color_g = Signal(8, reset=0)
        color_b = Signal(8, reset=0)

        got_hdr1 = Signal(reset=0)
        got_hdr2 = Signal(reset=0)
        byte_idx = Signal(2, reset=0)   # 0=R,1=G,2=B

        # ----- TX handshake for ACK 'K' -----
        pending = Signal(reset=0)
        tx_byte = Signal(8, reset=ord('K'))

        # default: deassert valid unless we have pending data
        self.sync += tx.valid.eq(0)

        self.sync += [
            # consume RX stream
            If(~got_hdr1,
                If(rx.valid & (rx.data == 0xAA), got_hdr1.eq(1))
            ).Elif(~got_hdr2,
                If(rx.valid,
                    If(rx.data == 0x55, got_hdr2.eq(1)).Else(got_hdr1.eq(0))
                )
            ).Else(
                # have header; collect 3 data bytes R,G,B
                If(rx.valid,
                    Case(byte_idx, {
                        0: [ color_r.eq(rx.data), byte_idx.eq(1) ],
                        1: [ color_g.eq(rx.data), byte_idx.eq(2) ],
                        2: [
                            color_b.eq(rx.data),
                            byte_idx.eq(0),
                            got_hdr1.eq(0),
                            got_hdr2.eq(0),
                            pending.eq(1)        # queue ACK after full packet
                        ],
                    })
                )
            ),

            # transmit ACK when pending, keep valid until accepted
            If(pending,
                tx.valid.eq(1),
                tx.data.eq(tx_byte),
                If(tx.ready, pending.eq(0))
            )
        ]

        # ----- simple 8-bit PWM on 3 LEDs (active-low) -----
        pwm_cnt = Signal(8)
        self.sync += pwm_cnt.eq(pwm_cnt + 1)

        # brightness: LED on while pwm_cnt < color
        # active-low LED pads -> invert
        self.comb += [
            leds[0].eq(~(pwm_cnt < color_r)),
            leds[1].eq(~(pwm_cnt < color_g)),
            leds[2].eq(~(pwm_cnt < color_b)),
        ]


# ---- integrate in your SoC ----
# after you create UARTPHY on custom pins:
#   self.submodules.uartphy = UARTPHY(pads, sys_clk_freq, baudrate=115200)
# request 3 LEDs (use any 3 you have):
#   leds = [platform.request("led_n", i) for i in range(3)]
#   self.submodules.rgb = RGBFromUART(self.uartphy, leds, sys_clk_freq)

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
        leds = [p.request("led_n", i) for i in range(3)]
        self.submodules.rgb = RGBFromUART(self.uartphy, leds, sys_clk_freq)
        

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