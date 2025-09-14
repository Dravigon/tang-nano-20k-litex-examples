#!/usr/bin/env python3
# Tang Nano 20K — SPI Slave: ALWAYS reply 0xA5 on every byte (Mode 0).
# CDC-safe: SCK/CS synchronized into sys clk; update MISO on SCK falling edge.
# Works for long CS windows (reloads 0xA5 every 8 bits).

from migen import *
from litex.gen import LiteXModule
from migen.genlib.cdc import MultiReg
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

# ---------- CRG ----------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(54e6)):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# ---------- SPI Slave: A5 repeat ----------
class SPISlaveA5Repeat(LiteXModule):
    def __init__(self, pads):
        # Pads
        sck_i  = pads.clk if hasattr(pads, "clk") else pads.sck
        mosi_i = pads.mosi  # unused, but keep requested
        miso_o = pads.miso
        csn_i  = pads.cs_n

        # Sync SCK/CS into sys clock domain
        sck_sync = Signal()
        csn_sync = Signal()
        self.specials += MultiReg(sck_i, sck_sync, odomain="sys")
        self.specials += MultiReg(csn_i, csn_sync, odomain="sys")

        # Edge detect (sys domain)
        sck_d = Signal()
        csn_d = Signal()
        self.sync += [sck_d.eq(sck_sync), csn_d.eq(csn_sync)]
        sck_fall = Signal()
        cs_act   = Signal()
        cs_fall  = Signal()
        self.comb += [
            sck_fall.eq(sck_d & ~sck_sync),  # 1 on SCK falling
            cs_act.eq(~csn_sync),           # CS active low
            cs_fall.eq(csn_d & ~csn_sync),  # CS falling
        ]

        # TX shift machinery for constant 0xA5
        tx_shift = Signal(8, reset=0xA5)   # current byte being shifted out
        miso     = Signal(reset=1)         # idle high
        bitcnt   = Signal(3)               # 0..7

        self.comb += miso_o.eq(miso)

        self.sync += [
            If(cs_fall,                    # new CS window
                tx_shift.eq(0xA5),
                bitcnt.eq(0),
                miso.eq(0xA5 >> 7)         # present MSB immediately
            ).Elif(cs_act,
                If(sck_fall,
                    # After each falling edge, move to next bit
                    # Present next MSB with ~half SCK before the rising sample (Mode 0).
                    miso.eq(tx_shift[6]),
                    tx_shift.eq(Cat(C(0, 1), tx_shift[:7])),  # left shift by 1 (LSB <- 0)
                    bitcnt.eq(bitcnt + 1),
                    If(bitcnt == 7,
                        # Byte boundary: reload A5 for the next byte (no CS toggle needed)
                        tx_shift.eq(0xA5),
                        bitcnt.eq(0),
                        miso.eq(0xA5 >> 7)
                    )
                )
            ).Else(
                miso.eq(1)                 # idle when not selected
            )
        ]

# ---------- Top SoC ----------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(54e6)):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(platform, sys_clk_freq=sys_clk_freq)
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         cpu_type=None, integrated_rom_size=0x0,
                         integrated_main_ram_size=0x1000)

        pads = platform.request("custom_spi", 0)  # your mapping (SCK=51, MOSI=54, MISO=56, CS_n=48)
        self.submodules.spi = SPISlaveA5Repeat(pads)

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
