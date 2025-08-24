#!/usr/bin/env python3
# Tang Nano 20K — Scanline-over-UART (RGB565) to HDMI 640x480
# Packet: AA 55 'R' y_lo y_hi  [1280 bytes RGB565 LE]  -> FPGA replies 'K'

from migen import *
from migen.genlib.cdc import AsyncResetSynchronizer, MultiReg
from litex.gen import *
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.soc.cores.uart  import UARTPHY
from litex.soc.interconnect import stream

from platforms import sipeed_tang_nano_20k

# ---------------- Config ----------------
HRES, VRES    = 640, 480
SYS_CLK_FREQ  = int(27e6)       # exact divisor for 1.5 Mbps (27e6 / 1.5e6 = 18)
PIXCLK_5X     = 125_875_000     # /5 => ~25.175 MHz
BAUD          = 1_500_000

# ---------------- CRG ----------------
class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst       = Signal()
        self.cd_sys    = ClockDomain()
        self.cd_por    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk27), por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        self.submodules.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIXCLK_5X, margin=1e-2)
        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5", i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk, o_CLKOUT=self.cd_hdmi.clk
        )

        # self.specials += [
        #     AsyncResetSynchronizer(self.cd_sys,    ~por_done | self.rst),
        #     AsyncResetSynchronizer(self.cd_hdmi,   ~por_done | self.rst),
        #     AsyncResetSynchronizer(self.cd_hdmi5x, ~por_done | self.rst),
        # ]

# -------------- Line buffer (sys write / hdmi read) --------------
class LinebufPorts:
    def __init__(self, mem, width):
        self.wr = mem.get_port(write_capable=True,  clock_domain="sys")
        self.rd = mem.get_port(                    clock_domain="hdmi")

# ----- UART row parser (sys) : AA 55 'R' y0 y1 + 1280B -> 'K' -----
class UART_RowToLinebuf(LiteXModule):
    def __init__(self, uartphy, width, line_wr_port):
        rx, tx = uartphy.source, uartphy.sink

        self.led_aa   = Signal()
        self.led_55   = Signal()
        self.rx_beat  = Signal()
        self.active   = Signal()

        # Always ready to accept RX bytes.
        self.comb += rx.ready.eq(1)

        # ---- LOCAL TX REGISTERS (single driver) ----
        tx_v = Signal(reset=0)
        tx_d = Signal(8)
        # Fanout to UART sink in ONE place (combinational)
        self.comb += [tx.valid.eq(tx_v), tx.data.eq(tx_d)]
        # Default TX valid low each cycle (single synchronous driver)
        self.sync += tx_v.eq(0)

        # Parser state
        got   = Signal(12)
        pixlo = Signal(8)
        x     = Signal(max=width+1)
        y     = Signal(16)

        fsm = FSM(reset_state="WAIT_A"); self.submodules += fsm

        fsm.act("WAIT_A",
            If(rx.valid & (rx.data == 0xAA),
                NextValue(self.led_aa, ~self.led_aa),
                NextState("WAIT_55")
            )
        )
        fsm.act("WAIT_55",
            If(rx.valid,
                If(rx.data == 0x55,
                    NextValue(self.led_55, ~self.led_55),
                    NextState("WAIT_R")
                ).Else(NextState("WAIT_A"))
            )
        )
        fsm.act("WAIT_R",
            If(rx.valid,
                If(rx.data == ord('R'), NextState("GET_Y0")).Else(NextState("WAIT_A"))
            )
        )
        fsm.act("GET_Y0",
            If(rx.valid, NextValue(y[0:8], rx.data), NextState("GET_Y1"))
        )
        fsm.act("GET_Y1",
            If(rx.valid,
                NextValue(y[8:16], rx.data),
                NextValue(got, 0), NextValue(x, 0),
                NextState("PAYLOAD")
            )
        )
        fsm.act("PAYLOAD",
            NextValue(self.active, 1),
            If(rx.valid,
                If(got < (width*2),
                    If((got & 1) == 0,
                        NextValue(pixlo, rx.data),
                        NextValue(got, got + 1)
                    ).Else(
                        NextValue(line_wr_port.adr,   x),
                        NextValue(line_wr_port.dat_w, Cat(pixlo, rx.data)),
                        NextValue(line_wr_port.we,    1),
                        NextValue(x, x + 1),
                        NextValue(got, got + 1)
                    )
                ).Else(
                    NextState("SEND_ACK")
                )
            )
        )
        fsm.act("SEND_ACK",
            # drive the *local* regs; UART sink is fed only from tx_v/tx_d above
            NextValue(tx_d, ord('K')),
            NextValue(tx_v, 1),
            If(tx.ready, NextState("WAIT_A"))
        )

        # Tick on each consumed byte (for LED)
        self.sync += If(rx.valid, self.rx_beat.eq(~self.rx_beat))
        # Single synchronous clear for write-enable unless we set it in PAYLOAD
        self.sync += If(~(fsm.ongoing("PAYLOAD") & rx.valid & (got[0] == 1)), line_wr_port.we.eq(0))


# ----------- Player: RGB565 -> RGB888 (hdmi) -----------
class ScanlinePlayer(LiteXModule):
    def __init__(self, width, vtg_sink, source, rd_port):
        self.vtg_sink = vtg_sink
        self.source   = source
        x     = Signal(max=width)
        de_z  = Signal(reset=0)
        first = Signal(reset=1)

        self.comb += rd_port.adr.eq(x)
        self.sync += [
            If(self.vtg_sink.valid & self.source.ready,
                If(self.vtg_sink.de,
                    If(x == (width-1), x.eq(0)).Else(x.eq(x + 1))
                ).Else(
                    x.eq(0), first.eq(1)
                ),
                If(~de_z & self.vtg_sink.de, first.eq(1)),
                If(de_z  & self.vtg_sink.de, first.eq(0)),
                de_z.eq(self.vtg_sink.de)
            )
        ]

        pix16 = rd_port.dat_r
        r5 = Signal(5); g6 = Signal(6); b5 = Signal(5)
        r8 = Signal(8); g8 = Signal(8); b8 = Signal(8)
        self.comb += [
            b5.eq(pix16[0:5]), g6.eq(pix16[5:11]), r5.eq(pix16[11:16]),
            r8.eq(Cat(r5, r5[2:5])), g8.eq(Cat(g6, g6[4:6])), b8.eq(Cat(b5, b5[2:5])),

            self.source.valid.eq(self.vtg_sink.valid),
            self.vtg_sink.ready.eq(self.source.ready),
            self.source.hsync.eq(self.vtg_sink.hsync),
            self.source.vsync.eq(self.vtg_sink.vsync),
            self.source.de.eq(self.vtg_sink.de & ~first),

            self.source.r.eq(Mux(self.source.de, r8, 0)),
            self.source.g.eq(Mux(self.source.de, g8, 0)),
            self.source.b.eq(Mux(self.source.de, b8, 0)),
        ]

# ----------------------------- Top --------------------------------
class Top(SoCCore):
    def __init__(self):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = CRG(platform, SYS_CLK_FREQ)

        SoCCore.__init__(self, platform, SYS_CLK_FREQ,
            cpu_type=None,
            integrated_rom_size=0x4000,
            integrated_main_ram_size=0x2000,
            with_uart=False
        )

        # HDMI PHY + VTG (hdmi)
        pads = platform.request("hdmi")
        class PadsWrap:
            def __init__(self, p):
                for n in ["clk_p","clk_n","data0_p","data0_n","data1_p","data1_n","data2_p","data2_n","sda","scl","cec"]:
                    if hasattr(p, n): setattr(self, n, getattr(p, n))
                self.hdp = 1
        pads_w = PadsWrap(pads)

        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(VideoGowinHDMIPHY(pads_w))
        self.submodules.vtg      = ClockDomainsRenamer("hdmi")(VideoTimingGenerator(default_video_timings="640x480@60Hz"))

        # Line buffer RAM (16b x 640): sys-write / hdmi-read
        mem = Memory(16, HRES); self.specials += mem
        ports = LinebufPorts(mem, HRES); self.specials += ports.wr, ports.rd

        # UART PHY (sys)
        pads_uart = platform.request("custom_serial")
        self.submodules.uartphy = UARTPHY(pads_uart, clk_freq=SYS_CLK_FREQ, baudrate=BAUD)

        # UART row parser (sys)
        self.submodules.rowrx = UART_RowToLinebuf(self.uartphy, HRES, ports.wr)

        # Player (hdmi)
        self.submodules.player = ClockDomainsRenamer("hdmi")(ScanlinePlayer(HRES, self.vtg.source, self.videophy.sink, ports.rd))

        # ---------------- LEDs (visible pulses) ----------------
        def pulse(strobe, bits=21):
            cnt = Signal(bits); out = Signal()
            self.sync += If(strobe, cnt.eq((1<<bits)-1)).Elif(cnt != 0, cnt.eq(cnt - 1))
            self.comb += out.eq(cnt != 0)
            return out

        # Heartbeats
        hb_sys = Signal(26); self.sync += hb_sys.eq(hb_sys + 1)
        self.comb += platform.request("led_n", 5).eq(~hb_sys[25])
        hb_pix = Signal(24); self.sync.hdmi += hb_pix.eq(hb_pix + 1)
        self.comb += platform.request("led_n", 4).eq(~hb_pix[23])

        # LED1: RX byte accepted (sys) — UARTPHY.source.valid when we read it (rx.ready=1)
        rx_byte = Signal()
        self.comb += rx_byte.eq(self.uartphy.source.valid)
        self.comb += platform.request("led_n", 1).eq(~pulse(rx_byte, bits=20))

        # LED2/LED3: header AA/55 pulses (sys)
        aa_t = Signal(); bb_t = Signal()
        self.sync += [
            If(self.rowrx.led_aa != aa_t, aa_t.eq(self.rowrx.led_aa)),
            If(self.rowrx.led_55 != bb_t, bb_t.eq(self.rowrx.led_55)),
        ]
        self.comb += platform.request("led_n", 2).eq(~pulse(self.rowrx.led_aa ^ aa_t, bits=20))
        self.comb += platform.request("led_n", 3).eq(~pulse(self.rowrx.led_55 ^ bb_t, bits=20))

        # LED0: row payload write activity (sys)
        self.comb += platform.request("led_n", 0).eq(~pulse(self.rowrx.active, bits=19))

def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = p.parse_args()
    soc  = Top()
    bld  = Builder(soc)
    if args.build: bld.build()
    if args.load:  soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
