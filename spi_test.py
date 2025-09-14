#!/usr/bin/env python3
# Tang Nano 20K — SPI Slave Echo (Mode 0, MSB-first), CDC-safe and slice-correct
# First byte after CS falling is 0xA5. Thereafter, echoes the previous RX byte.

from migen import *
from litex.gen import LiteXModule
from migen.genlib.cdc import MultiReg
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

# -------- CRG --------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(27e6)):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# --- SPI Slave Echo (MODE0) with proper edge timing and CS sync ---
from migen import *
from litex.gen import LiteXModule

class SPISlaveEcho_MODE0_SYS(LiteXModule):
    """
    SPI Slave (MODE0, MSB-first) in sys clock domain.
      - Sample MOSI on SCK rising.
      - Change MISO on SCK falling.
      - First byte after CS↓ : 0xA5
      - Thereafter           : echo previous received byte (1-byte latency).
    pads: .clk/.sck, .mosi, .miso, .cs_n (active-low)
    """
    def __init__(self, pads):
        # --- normalize pad names ---
        sck_i  = pads.clk if hasattr(pads, "clk") else pads.sck
        mosi_i = pads.mosi
        miso_o = pads.miso
        csn_i  = pads.cs_n

        # --- sync into sys domain ---
        sck_m = Signal(); sck = Signal(); sck_d = Signal()
        mosi_m= Signal(); mosi= Signal()
        cs_m  = Signal(reset=1); cs = Signal(reset=1); cs_d = Signal(reset=1)

        self.sync += [
            sck_m.eq(sck_i),  sck.eq(sck_m),  sck_d.eq(sck),
            mosi_m.eq(mosi_i), mosi.eq(mosi_m),
            cs_m.eq(csn_i),   cs.eq(cs_m),    cs_d.eq(cs),
        ]

        cs_active = ~cs
        sck_rise  = ~sck_d & sck
        sck_fall  =  sck_d & ~sck
        cs_fall   =  cs_d  & ~cs
        cs_rise   = ~cs_d  &  cs

        # --- datapath ---
        bitcnt   = Signal(3)
        rx_shift = Signal(8)                 # assemble RX MSB-first
        tx_shift = Signal(8, reset=0xA5)     # shift out MSB-first
        tx_next  = Signal(8, reset=0xA5)     # next byte to load after a full RX
        load_tx  = Signal()                  # one-cycle strobe when a byte completes
        miso_q   = Signal(reset=1)

        # drive MISO (idle high when not selected)
        self.comb += miso_o.eq(Mux(cs_active, miso_q, 1))

        # (re)select: preload first reply and present its MSB immediately
        self.sync += If(cs_fall,
            bitcnt.eq(0),
            rx_shift.eq(0),
            tx_next.eq(0xA5),
            tx_shift.eq(0xA5),
            miso_q.eq((0xA5 >> 7) & 1),      # present MSB
            load_tx.eq(0)
        ).Elif(cs_rise,
            load_tx.eq(0),
            bitcnt.eq(0)
        )

        self.sync += If(cs_active,
            # Sample MOSI on rising edge — **MSB-first assembly**
            # rx_shift = (rx_shift << 1) | mosi
            If(sck_rise,
                rx_shift.eq(Cat(mosi, rx_shift[:-1])),   # LSB<=mosi, others shift up
                If(bitcnt == 7,
                    tx_next.eq(Cat(mosi, rx_shift[:-1])),# use freshly completed byte
                    bitcnt.eq(0),
                    load_tx.eq(1)
                ).Else(
                    bitcnt.eq(bitcnt + 1),
                    load_tx.eq(0)
                )
            ),

            # Drive MISO on falling edge — **MSB-first shift out**
            If(sck_fall,
                If(load_tx,
                    tx_shift.eq(tx_next),
                    miso_q.eq(tx_next[7]),
                    load_tx.eq(0)
                ).Else(
                    miso_q.eq(tx_shift[6]),
                    tx_shift.eq(Cat(Constant(0,1), tx_shift[:-1]))  # shift left, keep MSB-first
                )
            )
        ).Else(
            miso_q.eq(1),
            load_tx.eq(0)
        )


# -------- Top SoC --------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(27e6)):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(platform, sys_clk_freq=sys_clk_freq)
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0x0,
                         integrated_main_ram_size=0x1000)
        pads = platform.request("custom_spi", 0)  # uses your platform pin map
        self.submodules.spi = SPISlaveEcho_MODE0_SYS(pads)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--sys-clk-freq", default=324e6, type=float,
                               help="Try 162e6; fallback 135e6/108e6 if timing fails.")
    # parser.add_target_argument("--build", action="store_true")
    # parser.add_target_argument("--load",  action="store_true")
    args = parser.parse_args()

    soc = Top(sys_clk_freq=int(float(args.sys_clk_freq)))
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(
            builder.get_bitstream_filename(mode="sram")
        )

if __name__ == "__main__":
    main()
