from migen import Signal
from litex.soc.integration.soc_core import SoCCore
from litescope import LiteScopeAnalyzer
from litex.build.io import LED

# Define a simple SoC that includes LiteScope
class MySoC(SoCCore):
    def __init__(self, platform):
        # Set up the clock frequency
        clk_freq = 100e6  # 100 MHz for example

        # Initialize SoC
        SoCCore.__init__(self, platform, clk_freq)

        # Create a counter to monitor
        self.counter = Signal(32)
        self.submodules.counter_logic = counter_logic = Counter()

        # Set up LiteScope
        self.submodules.analyzer = LiteScopeAnalyzer([self.counter],
            depth        = 512,
            clock_domain = "sys",
            samplerate   = clk_freq,
            csr_csv      = "analyzer.csv"
        )

        # Set up the counter logic (simple counter)
        self.sync += self.counter.eq(self.counter + 1)

# Define the counter logic (simple counter)
class Counter(Module):
    def __init__(self):
        self.counter = Signal(32)
        self.sync += self.counter.eq(self.counter + 1)

# Create platform instance (adjust for your platform)
from litex_boards.platforms import arty
platform = arty.Platform()

# Create SoC instance and build
soc = MySoC(platform)
soc.build()

from litescope import LiteScopeAnalyzer
