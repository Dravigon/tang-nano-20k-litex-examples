#!/usr/bin/env python3

from migen import *
from litex.gen import *
from litex_boards.platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.build.generic_platform import Pins, IOStandard

# Import the original _connectors mapping
from litex_boards.platforms.sipeed_tang_nano_20k import _connectors

# === Choose which header + pin index to test ===
HEADER_NAME = "J5"  # "J5" or "J6"
PIN_INDEX   = 9     # index in the connector mapping (0-based)
# ================================================

class PinVoltageTest(Module):
    def __init__(self, test_pin, led_pin):
        # LED is active-low -> invert
        self.comb += led_pin.eq(~test_pin)

class BaseSoC(SoCCore):
    def __init__(self, **kwargs):
        platform = sipeed_tang_nano_20k.Platform()

        # Look up the pin from _connectors
        package_pin = None
        for name, pin_list in _connectors:
            if name == HEADER_NAME:
                pins = pin_list.split()
                package_pin = pins[PIN_INDEX]
                break

        if package_pin is None or package_pin == "-":
            raise ValueError(f"No valid pin at {HEADER_NAME} index {PIN_INDEX}")

        # Add as a temporary resource
        platform.add_extension([
            ("probe_pin", 0, Pins(package_pin), IOStandard("LVCMOS33"))
        ])

        test_pin = platform.request("probe_pin", 0)
        led_pin  = platform.request("led_n", 0)

        # Minimal SoC
        SoCCore.__init__(self, platform, clk_freq=27e6,
                         cpu_type=None, integrated_rom_size=0, integrated_sram_size=0,
                         **kwargs)

        self.submodules.test = PinVoltageTest(test_pin, led_pin)

if __name__ == "__main__":
    soc = BaseSoC()
    builder = Builder(soc, compile_software=False)
    builder.build()
    soc.platform.create_programmer().load_bitstream(
        builder.get_bitstream_filename(mode="sram")
    )
