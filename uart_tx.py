from migen import *
from litex.gen import *
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.build.io import DDROutput
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *


from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY

from platforms import sipeed_tang_nano_20k
from litex.build.generic_platform import *



class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_hdmi=False):
        self.rst = Signal()
        self.cd_sys = ClockDomain()
        self.cd_por = ClockDomain()
        if with_hdmi:
            self.cd_hdmi = ClockDomain()
            self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")
        por_count = Signal(16, reset=2**16-1)
        por_done = Signal()
        self.comb += self.cd_por.clk.eq(clk27)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        self.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        if with_hdmi:
            self.video_pll = video_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
            video_pll.register_clkin(clk27, 27e6)
            video_pll.create_clkout(self.cd_hdmi5x, 125e6, margin=1e-2)
            self.specials += Instance("CLKDIV",
                p_DIV_MODE="5",
                i_RESETN=1,
                i_CALIB=0,
                i_HCLKIN=self.cd_hdmi5x.clk,
                o_CLKOUT=self.cd_hdmi.clk
            )


class MySoC(SoCCore):
    def __init__(self, toolchain="gowin", sys_clk_freq=48e6, with_video_colorbars=False, **kwargs):
        platform = sipeed_tang_nano_20k.Platform(toolchain=toolchain)
        self.crg = _CRG(platform, sys_clk_freq, with_hdmi=with_video_colorbars)

        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Tang Nano 20K", **kwargs)


        if not self.integrated_main_ram_size:
            class SDRAMPads:
                def __init__(self):
                    p = platform
                    self.clk = p.request("O_sdram_clk")
                    self.cke = p.request("O_sdram_cke")
                    self.cs_n = p.request("O_sdram_cs_n")
                    self.cas_n = p.request("O_sdram_cas_n")
                    self.ras_n = p.request("O_sdram_ras_n")
                    self.we_n = p.request("O_sdram_wen_n")
                    self.dm = p.request("O_sdram_dqm")
                    self.a = p.request("O_sdram_addr")
                    self.ba = p.request("O_sdram_ba")
                    self.dq = p.request("IO_sdram_dq")
            pads = SDRAMPads()
            self.specials += DDROutput(0, 1, pads.clk, ClockSignal("sys"))
            self.sdrphy = GENSDRPHY(pads, sys_clk_freq)
            self.add_sdram("sdram", phy=self.sdrphy, module=M12L64322A(sys_clk_freq, "1:1"), l2_cache_size=128)

      
        self.add_uart("serial0", "custom_serial")
        self.send_message("Hello, UART!")

    def send_message(self, message):
        """
        This method sends a message via UART (USB via UART bridge).
        It sends one byte at a time.
        """
        for char in message:
            self.send_byte(ord(char))

    def send_byte(self, byte):
        """
        This method sends a single byte over UART.
        """
        # Trigger the UART to send the byte (1 byte at a time)
        self.sync += [
            If(self.uart.sink.ready,
               self.uart.sink.valid.eq(1),       # Set the sink as valid
               self.uart.sink.data.eq(byte)      # Send the byte
            ).Else(
                self.uart.sink.valid.eq(0)       # Deassert sink when not ready
            )
        ]


def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--with-video-colorbars", default=True,action="store_true")
    parser.add_target_argument("--sys-clk-freq", default=27e6, type=float, help="System clock frequency.")
    args = parser.parse_args()

    soc = MySoC(
        toolchain=args.toolchain,
        sys_clk_freq=args.sys_clk_freq,
        with_video_colorbars=args.with_video_colorbars,
        **parser.soc_argdict
    )

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
