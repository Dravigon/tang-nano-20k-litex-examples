from migen import *
from litex_boards.platforms import sipeed_tang_nano_20k

# Button toggles LED.
class ButtonToLed(Module):
    def __init__(self, platform):
        btn = platform.request("btn", 0)       # Button (active High)
        led = platform.request("led_n", 1)     # LED (active low)

        state = Signal(reset=0)
        counter = Signal(24)

        self.sync += counter.eq(counter + 1)

        self.sync += [
            If(counter == 0,
                If(btn,       # Active High
                    state.eq(~state)
                )
            )
        ]

        self.comb += led.eq(~state)

# Build + Flash
def main():
    from litex.build.tools import write_to_file
    from litex.build.openfpgaloader import OpenFPGALoader

    platform = sipeed_tang_nano_20k.Platform()
    top = ButtonToLed(platform)
    platform.build(top, build_name="button_led")

    prog = OpenFPGALoader(cable="ft2232")
    prog.load_bitstream("build/button_led.fs")

if __name__ == "__main__":
    main()
