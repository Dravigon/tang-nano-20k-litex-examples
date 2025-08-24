from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import MultiReg

from litex.gen import *
from litex.build.io import DDROutput
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.cores.gpio import GPIOIn
from litex.soc.cores.uart import UARTPHY, UART

from litex.soc.cores.led import LedChaser, WS2812
from litex.soc.cores.video import VideoGowinHDMIPHY
from litex.soc.interconnect import stream

from litedram.modules import M12L64322A
from litedram.phy import GENSDRPHY

from platforms import sipeed_tang_nano_20k


# Constants
hbits = 12
vbits = 12

video_timing_layout = [
    ("hsync", 1), ("vsync", 1), ("de", 1),
    ("hres", hbits), ("vres", vbits),
    ("hcount", hbits), ("vcount", vbits),
]

video_data_layout = [
    ("hsync", 1), ("vsync", 1), ("de", 1),
    ("r", 8), ("g", 8), ("b", 8),
]

class ColorBarsPattern(LiteXModule):
    def __init__(self,platform,r_sys,g_sys,b_sys):
        self.enable = Signal(reset=1)
        self.vtg_sink = vtg_sink = stream.Endpoint(video_timing_layout)
        self.source = source = stream.Endpoint(video_data_layout)

        enable = Signal()
        self.specials += MultiReg(self.enable, enable)

        pix = Signal(hbits)
        bar = Signal(3)

        btn = platform.request("btn", 0)       # Button (active High)
        led = platform.request("led_n", 2)     # LED (active low)

        state = Signal(reset=0)
        counter = Signal(24)

        self.sync += counter.eq(counter + 1)

        self.sync += [
            vtg_sink.connect(source, keep={"valid", "ready", "last", "de", "hsync", "vsync"}),
            source.r.eq(r_sys),
            source.g.eq(g_sys),
            source.b.eq(b_sys),
            # If(counter == 0,
            #     If(btn,       # Active High
            #         [state.eq(~state),
            #         bar.eq(bar + 1)]
            #     )
            # )
        ]

        # self.comb += led.eq(~state)

        # color_bar = [
        #     [0xff, 0xff, 0xff], [0xff, 0xff, 0x00],
        #     [0x00, 0xff, 0xff], [0x00, 0xff, 0x00],
        #     [0xff, 0x00, 0xff], [0xff, 0x00, 0x00],
        #     [0x00, 0x00, 0xff], [0x00, 0x00, 0x00]
        # ]
        # cases = {}
        # for i, (r, g, b) in enumerate(color_bar):
        #     cases[i] = [source.r.eq(r), source.g.eq(g), source.b.eq(b)]
        # self.comb += Case(bar, cases)

        # self.comb += [
        #     source.r.eq(r_sys),
        #     source.g.eq(g_sys),
        #     source.b.eq(b_sys),
        # ]



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

class UartColorRX(LiteXModule):
    def __init__(self, pads, led_n_byte, led_n_heartbeat, sys_clk_freq=27e6, baud=115200):
        from litex.soc.cores.uart import UARTPHY, UART
        # UART PHY/Core in sys domain
        self.submodules.uart_phy = UARTPHY(pads, sys_clk_freq, baudrate=baud)
        self.submodules.uart     = UART(self.uart_phy)

        # Always ready to accept RX stream bytes
        self.comb += self.uart.source.ready.eq(1)

        # Rising-edge detect on "valid"
        prev_valid = Signal(reset=0)
        byte_pulse = Signal()
        self.comb += byte_pulse.eq(self.uart.source.valid & ~prev_valid)
        self.sync += prev_valid.eq(self.uart.source.valid)

        # Output registers: current color (in sys domain)
        self.r_sys = Signal(8, reset=0)
        self.g_sys = Signal(8, reset=0)
        self.b_sys = Signal(8, reset=0)

        blinker = Signal(1,reset=0)
        blinker.eq(1)
        flash_cnt = Signal(22)  # ~50ms @27 MHz


                # ------------- RX diagnostics: raw edge detect -------------
        # Sync the async RX pin then detect edges to confirm wiring
        rx_sync1 = Signal()
        rx_sync2 = Signal()
        rx_prev  = Signal()
        rx_edge  = Signal()  # 1 cycle on any transition

        self.sync += [
            rx_sync1.eq(pads.rx),
            rx_sync2.eq(rx_sync1),
            rx_prev.eq(rx_sync2),
        ]
        self.comb += rx_edge.eq(rx_sync2 ^ rx_prev)


        # Simple packet parser: 0xA5 R G B
        SYNC  = 0
        RED   = 1
        GREEN = 2
        BLUE  = 3
        state = Signal(2, reset=SYNC)




        self.sync += [
            If( rx_edge,           # any RX activity OR a good byte
                flash_cnt.eq(int(27e6*0.05)),
                blinker.eq(1)
            ).Elif(blinker,
                If(flash_cnt != 0,
                    flash_cnt.eq(flash_cnt - 1)
                ).Else(
                    blinker.eq(0)
                )
            )
        ]


        self.sync += If(byte_pulse,
            Case(state, {
                SYNC:  If(self.uart.source.data == 0xA5, [state.eq(RED)]).Else([state.eq(SYNC)]),
                RED:   [ self.r_sys.eq(self.uart.source.data), state.eq(GREEN) ],
                GREEN: [ self.g_sys.eq(self.uart.source.data), state.eq(BLUE)  ],
                BLUE:  [ self.b_sys.eq(self.uart.source.data), state.eq(SYNC)  ],
            })
        )

class UARTBlink(LiteXModule):
    def __init__(self, pads, led_n_byte, led_n_heartbeat, sys_clk_freq):

        # UART
        self.submodules.uart_phy = UARTPHY(pads, sys_clk_freq, baudrate=115200)
        self.submodules.uart     = UART(self.uart_phy)

        # Always ready to accept RX bytes
        self.comb += self.uart.source.ready.eq(1)

        # ------------- Heartbeat (~1 Hz) on led_n[1] -------------
        hb_cnt = Signal(25)
        self.sync += hb_cnt.eq(hb_cnt + 1)
        self.comb += led_n_heartbeat.eq(~hb_cnt[24])  # active-low LED

        # ------------- RX diagnostics: raw edge detect -------------
        # Sync the async RX pin then detect edges to confirm wiring
        rx_sync1 = Signal()
        rx_sync2 = Signal()
        rx_prev  = Signal()
        rx_edge  = Signal()  # 1 cycle on any transition

        self.sync += [
            rx_sync1.eq(pads.rx),
            rx_sync2.eq(rx_sync1),
            rx_prev.eq(rx_sync2),
        ]
        self.comb += rx_edge.eq(rx_sync2 ^ rx_prev)

        # ------------- Flash LED on (a) RX edge, (b) valid byte -------------
        flash_cnt = Signal(22)  # ~50ms @27 MHz
        flashing  = Signal()

        # Rising-edge detect on RX-valid
        prev_valid = Signal(reset=0)
        byte_pulse = Signal()
        # self.comb += byte_pulse.eq(self.uart.source.valid & ~prev_valid)
        # self.sync += prev_valid.eq(self.uart.source.valid)

        # self.sync += [
        #     If(rx_edge ,           # any RX activity OR a good byte
        #         flash_cnt.eq(int(27e6*0.05)),
        #         flashing.eq(1)
        #     ).Elif(flashing,
        #         If(flash_cnt != 0,
        #             flash_cnt.eq(flash_cnt - 1)
        #         ).Else(
        #             flashing.eq(0)
        #         )
        #     )
        # ]



        # Generate a single cycle pulse when a new byte is detected
        self.comb += [
            If(self.uart.source.valid & ~prev_valid,   # Detect rising edge of `valid`
                byte_pulse.eq(1)  # Assert the pulse
            ).Else(
                byte_pulse.eq(0)  # Reset pulse after one cycle
            ),
            prev_valid.eq(self.uart.source.valid)  # Store the previous state of `valid`
        ]


        # LED blinking logic
        self.sync += [
            If(byte_pulse,  # Trigger the flashing on new byte received
                flash_cnt.eq(int(27e6 * 0.05)),  # Flash for a small duration (adjust as needed)
                flashing.eq(1)
            ).Elif(flashing,  # If flashing is in progress
                If(flash_cnt != 0,
                    flash_cnt.eq(flash_cnt - 1)  # Decrease the flash count
                ).Else(
                    flashing.eq(0)  # Stop flashing when counter reaches zero
                )
            )
        ]




        self.comb += led_n_byte.eq(~flashing)  # active-low LED
        self.rx_byte_pulse = Signal()
        self.comb += self.rx_byte_pulse.eq(flashing)

        # ------------- TX diagnostic: send 0x55 periodically -------------
        tx_tick_cnt = Signal(25)  # about 0.8s at 27 MHz -> adjust if you want faster
        self.sync += tx_tick_cnt.eq(tx_tick_cnt + 1)
        tx_tick = Signal()
        self.comb += tx_tick.eq(tx_tick_cnt == 0)

        # regs to drive sink (avoid procedural assigns to wires)
        tx_valid_r = Signal(reset=0)
        tx_data_r  = Signal(8, reset=0x55)

        self.comb += [
            self.uart.sink.valid.eq(tx_valid_r),
            self.uart.sink.data.eq(tx_data_r),
        ]

        self.sync += [
            # On periodic tick, request to send 0x55 if we're idle
            If(tx_tick & ~tx_valid_r,
                tx_data_r.eq(0x55),
                tx_valid_r.eq(1)
            ).Elif(self.uart.sink.ready,
                tx_valid_r.eq(0)
            )
        ]


def add_video_colorbars(soc, phy, timings="800x600@60Hz", clock_domain="sys",platform={},r_sys={},g_sys={},b_sys={}):
    from litex.soc.cores.video import VideoTimingGenerator

    vtg = VideoTimingGenerator(default_video_timings=timings)
    vtg = ClockDomainsRenamer(clock_domain)(vtg)
    soc.add_module(name="video_vtg", module=vtg)

    colorbars = ClockDomainsRenamer(clock_domain)(ColorBarsPattern(platform,r_sys,g_sys,b_sys))
    soc.add_module(name="video_colorbars", module=colorbars)

    soc.comb += [
        vtg.source.connect(colorbars.vtg_sink),
        colorbars.source.connect(phy.sink)
    ]

class BaseSoC(SoCCore):
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

        #Debug
        # uart_pads = platform.request("serial")
        # self.submodules.uart_phy = UARTPHY(pads, sys_clk_freq, baudrate=115200)
        # self.submodules.uart     = UART(self.uart_phy)


        # PADS
        uart_pads = platform.request("custom_serial")  # your mapping: TX=48, RX=51
        led_byte  = platform.request("led_n", 0)       # flashes on byte
        led_hb    = platform.request("led_n", 1)       # heartbeat

        # Logic
        self.submodules.uart_blink = UARTBlink(uart_pads, led_byte, led_hb, sys_clk_freq)

        # self.submodules.uart_rgb = UartColorRX(uart_pads, led_byte, led_hb, sys_clk_freq)

        if with_video_colorbars:
            self.videophy = VideoGowinHDMIPHY(platform.request("hdmi"), clock_domain="hdmi")
            # add_video_colorbars(self, phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi",platform=self.platform, 
            #                     r_sys=self.uart_rgb.r_sys,
            #                     g_sys=self.uart_rgb.g_sys,
            #                     b_sys=self.uart_rgb.b_sys)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--with-video-colorbars", default=True,action="store_true")
    parser.add_target_argument("--sys-clk-freq", default=27e6, type=float, help="System clock frequency.")
    args = parser.parse_args()

    soc = BaseSoC(
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
