#!/usr/bin/env python3
# Tang Nano 20K - SDRAM write/read self-test (no CPU, LED status)

from migen import *
from litex.gen import LiteXModule
from litex.build.io import DDROutput

from litex_boards.platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.interconnect import wishbone

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY


# ------------------- CRG -------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(48e6)):
        self.cd_sys = ClockDomain()
        clk27 = platform.request("clk27")
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)


# ------------------- SDRAM Pads wrapper -------------------
class _SDRAMPads:
    def __init__(self, platform):
        self.clk   = platform.request("O_sdram_clk")
        self.cke   = platform.request("O_sdram_cke")
        self.cs_n  = platform.request("O_sdram_cs_n")
        self.cas_n = platform.request("O_sdram_cas_n")
        self.ras_n = platform.request("O_sdram_ras_n")
        self.we_n  = platform.request("O_sdram_wen_n")
        self.dm    = platform.request("O_sdram_dqm")
        self.a     = platform.request("O_sdram_addr")
        self.ba    = platform.request("O_sdram_ba")
        self.dq    = platform.request("IO_sdram_dq")


# ------------------- SDRAM Tester (WB master) -------------------
class SDRAMTester(LiteXModule):
    """Write pattern to SDRAM then read/verify."""
    def __init__(self, length_words=64*1024, base_addr=0):
        self.length_words = length_words
        self.base_addr    = base_addr

        # 32-bit Wishbone master, address in words
        self.bus = wishbone.Interface(data_width=32, adr_width=30)

        self.done   = Signal(reset=0)
        self.errors = Signal(32, reset=0)

        aw     = len(self.bus.adr)
        addr   = Signal(aw)
        addr32 = Signal(32)
        patt   = Signal(32)
        baddr  = Signal(aw)

        self.comb += [
            baddr.eq((self.base_addr >> 2) & ((1 << aw) - 1)),
            addr32.eq(Cat(addr, Replicate(0, 32 - aw))),
            patt.eq(addr32 ^ 0xA5A5A5A5),
        ]

        def wb_defaults():
            return [
                self.bus.cyc.eq(1),
                self.bus.stb.eq(1),
                self.bus.sel.eq(0xF),
                self.bus.adr.eq(baddr + addr),
            ]

        fsm = FSM(reset_state="WRITE")
        self.submodules += fsm

        fsm.act("WRITE",
            *wb_defaults(),
            self.bus.we.eq(1),
            self.bus.dat_w.eq(patt),
            If(self.bus.ack,
                If(addr == (self.length_words - 1),
                    NextValue(addr, 0),
                    NextState("READ")
                ).Else(
                    NextValue(addr, addr + 1)
                )
            )
        )

        fsm.act("READ",
            *wb_defaults(),
            self.bus.we.eq(0),
            If(self.bus.ack,
                If(self.bus.dat_r != patt,
                    NextValue(self.errors, self.errors + 1)
                ),
                If(addr == (self.length_words - 1),
                    NextState("DONE")
                ).Else(
                    NextValue(addr, addr + 1)
                )
            )
        )

        fsm.act("DONE",
            self.done.eq(1)
        )


# ------------------- SoC -------------------
class SDRAMRWTestSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(48e6), test_length_words=64*1024, **kwargs):
        # No CPU; LED-only status
        kwargs.setdefault("cpu_type", None)
        kwargs.setdefault("integrated_rom_size", 0)
        kwargs.setdefault("integrated_sram_size", 0x1000)
        kwargs.setdefault("with_uart", False)

        platform = sipeed_tang_nano_20k.Platform()

        # --- FIX: use 'clk_freq' (and keep a fallback for older/newer LiteX) ---
        try:
            SoCCore.__init__(self, platform,
                             clk_freq=sys_clk_freq,
                             ident="Tang Nano 20K - SDRAM R/W Test (no CPU)",
                             **kwargs)
        except TypeError:
            # Some trees still want positional clk_freq
            SoCCore.__init__(self, platform, sys_clk_freq,
                             ident="Tang Nano 20K - SDRAM R/W Test (no CPU)",
                             **kwargs)

        # Clock/Reset
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SDRAM PHY + DDR clock to chip
        pads = _SDRAMPads(platform)
        self.specials += DDROutput(0, 1, pads.clk, ClockSignal("sys"))
        self.submodules.sdrphy = GENSDRPHY(pads, sys_clk_freq)

        self.add_sdram(
            "sdram",
            phy           = self.sdrphy,
            module        = M12L64322A(sys_clk_freq, "1:1"),
            l2_cache_size = 128
        )

        # Tester WB master hooked to main RAM
        main_ram_origin = self.bus.regions["main_ram"].origin
        self.submodules.tester = SDRAMTester(
            length_words = test_length_words,
            base_addr    = main_ram_origin
        )
        self.add_wb_master(self.tester.bus)

        # LEDs (active-low)
        leds = platform.request_all("led_n")
        pass_led = leds[0]
        err_led  = leds[1] if len(leds) > 1 else None

        blink = Signal(24)
        self.sync += blink.eq(blink + 1)

        # PASS: solid ON when done & no errors
        self.comb += pass_led.eq(~(self.tester.done & (self.tester.errors == 0)))

        # ERROR: blink if any error
        if err_led is not None:
            self.comb += err_led.eq(~((self.tester.errors != 0) & blink[22]))


# ------------------- Build/Load -------------------
def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform,
                            description="Tang Nano 20K SDRAM R/W self-test (no CPU).")
    p.add_target_argument("--sys-clk-freq", default=48e6, type=float, help="System clock frequency.")
    p.add_target_argument("--test-length-words", default=64*1024, type=int,
                          help="Number of 32-bit words to write/read.")
    p.add_target_argument("--flash", action="store_true", help="Flash bitstream to SPI flash.")
    args = p.parse_args()

    soc = SDRAMRWTestSoC(sys_clk_freq=int(args.sys_clk_freq),
                         test_length_words=args.test_length_words,
                         **p.soc_argdict)
    builder = Builder(soc, **p.builder_argdict)

    if args.build:
        builder.build(**p.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        # Some LiteX versions expose only one filename method; try SRAM first.
        try:
            bit = builder.get_bitstream_filename(mode="sram")
        except TypeError:
            bit = builder.get_bitstream_filename()
        prog.load_bitstream(bit)

    if args.flash:
        prog = soc.platform.create_programmer()
        fs = builder.get_bitstream_filename(mode="flash", ext=".fs")
        prog.flash(0, fs, external=True)


if __name__ == "__main__":
    main()
