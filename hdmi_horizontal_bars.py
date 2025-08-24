#!/usr/bin/env python3
# Tang Nano 20K — HDMI 640x480 color bars (no UART)
# - Keeps HDMI/CRG style from your known-good hdmi_scan_line.py
# - Two line buffers (16b x 640) ping-pong each line: sys writes next line,
#   hdmi reads current line. Colors are chosen from vcount thresholds.

from migen import *
from migen.genlib.cdc import MultiReg
from litex.gen import LiteXModule
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder  import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.cores.video import VideoTimingGenerator, VideoGowinHDMIPHY
from litex.gen.genlib.cdc import ClockDomainsRenamer

# Platform (your local file)
from platforms import sipeed_tang_nano_20k

# ---------------- Config (match your working HDMI setup) ----------------
HRES, VRES    = 640, 480
SYS_CLK_FREQ  = int(27e6)         # keep it simple @ 27 MHz sys
PIXCLK_5X     = 125_875_000       # /5 -> ~25.175 MHz pixel clock

# ---------------- CRG ----------------
class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys    = ClockDomain()
        self.cd_hdmi   = ClockDomain()
        self.cd_hdmi5x = ClockDomain()

        clk27 = platform.request("clk27")

        # Sys PLL
        self.submodules.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        pll.register_clkin(clk27, 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # HDMI PLL *5 then /5
        self.submodules.vpll = vpll = GW2APLL(devicename=platform.devicename, device=platform.device)
        vpll.register_clkin(clk27, 27e6)
        vpll.create_clkout(self.cd_hdmi5x, PIXCLK_5X, margin=1e-2)

        self.specials += Instance("CLKDIV",
            p_DIV_MODE="5", i_RESETN=1, i_CALIB=0,
            i_HCLKIN=self.cd_hdmi5x.clk, o_CLKOUT=self.cd_hdmi.clk
        )

# --------- 16-bit RGB565 -> 8:8:8 ----------
def rgb565_to_888(p16):
    r5 = p16[11:16]; g6 = p16[5:11]; b5 = p16[0:5]
    r8 = Cat(r5, r5[2:5])
    g8 = Cat(g6, g6[4:6])
    b8 = Cat(b5, b5[2:5])
    return r8, g8, b8

# --------- Line buffer helper (dual-port BRAM: sys write, hdmi read) ---------
class LinebufPorts:
    def __init__(self, mem):
        # Write port (sys)
        self.wr = mem.get_port(write_capable=True,  clock_domain="sys")
        # Read port  (hdmi)
        self.rd = mem.get_port(                    clock_domain="hdmi")

# ------------------- Bars writer (sys) ----------------------
class BarsLineWriter(LiteXModule):
    """
    On each req toggle, fills the *next* line buffer with a solid color from req_y.
    buf_sel_sys tells which buffer HDMI reads this line -> we write the other one.
    """
    def __init__(self, wr0, wr1):
        self.req_y       = Signal(16)  # requested line index (from HDMI)
        self.req_toggle  = Signal()    # toggles each line start (from HDMI)
        self.buf_sel_sys = Signal()    # 0: HDMI reads buf0 now, so we write buf1; 1: opposite

        # Edge detect on toggle
        toggle_d = Signal()
        new_req  = Signal()
        self.sync += toggle_d.eq(self.req_toggle)
        self.comb += new_req.eq(self.req_toggle ^ toggle_d)

        # Choose which WR port to use for this fill
        use_wr1 = Signal()
        self.comb += use_wr1.eq(~self.buf_sel_sys)  # if HDMI reads 0, we write 1; if reads 1, write 0

        # Per-line write control
        x         = Signal(max=HRES)
        wr_active = Signal(reset=0)
        color     = Signal(16)

        # y -> color (8 bars, 60-line thresholds)
        # Do it at line start (on new_req)
        self.sync += [
            If(new_req,
                wr_active.eq(1),
                x.eq(0),
                If(self.req_y >= 420, color.eq(0x0000)).Elif(
                self.req_y >= 360, color.eq(0xFFFF)).Elif(
                self.req_y >= 300, color.eq(0x07FF)).Elif(
                self.req_y >= 240, color.eq(0xF81F)).Elif(
                self.req_y >= 180, color.eq(0xFFE0)).Elif(
                self.req_y >= 120, color.eq(0x001F)).Elif(
                self.req_y >=  60, color.eq(0x07E0)).Else(
                                   color.eq(0xF800))
            ).Elif(wr_active,
                If(x == (HRES - 1),
                    wr_active.eq(0)
                ).Else(
                    x.eq(x + 1)
                )
            )
        ]

        # Drive both BRAM write ports; gate WE so only one is active
        self.comb += [
            wr0.adr.eq(x),
            wr1.adr.eq(x),
            wr0.dat_w.eq(color),
            wr1.dat_w.eq(color),
            wr0.we.eq(Mux(use_wr1, 0,         wr_active)),
            wr1.we.eq(Mux(use_wr1, wr_active, 0)),
        ]

# ------------------ HDMI scanout (hdmi domain) ---------------------
class Scanout(LiteXModule):
    """
    Reads the selected buffer (buf_sel_hdmi) and feeds the HDMI PHY.
    """
    def __init__(self, rd0, rd1, vtg, sink, buf_sel_hdmi):
        x = Signal(max=HRES)

        # Address both read ports with x
        self.comb += [rd0.adr.eq(x), rd1.adr.eq(x)]

        # Which buffer does HDMI read this line?
        use_rd0 = Signal()
        self.comb += use_rd0.eq(~buf_sel_hdmi)  # 0 -> rd0, 1 -> rd1

        # Advance x only while vtg.valid & sink.ready; reset outside DE
        self.sync += [
            If(vtg.valid & sink.ready,
                If(vtg.de,
                    If(x == (HRES - 1), x.eq(0)).Else(x.eq(x + 1))
                ).Else(
                    x.eq(0)
                )
            )
        ]

        pix16      = Signal(16)
        self.sync += If(vtg.de, pix16.eq(Mux(use_rd0, rd0.dat_r, rd1.dat_r))).Else(pix16.eq(0))
        r8, g8, b8 = rgb565_to_888(pix16)

        # Handshake and drive PHY sink
        self.comb += [
            sink.valid.eq(vtg.valid),
            vtg.ready.eq(sink.ready),
            sink.hsync.eq(vtg.hsync),
            sink.vsync.eq(vtg.vsync),
            sink.de.eq(vtg.de),
            sink.r.eq(Mux(sink.de, r8, 0)),
            sink.g.eq(Mux(sink.de, g8, 0)),
            sink.b.eq(Mux(sink.de, b8, 0)),
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

        # HDMI PHY + VTG (wrap like your working code)
        pads = platform.request("hdmi")
        class PadsWrap:
            def __init__(self, p):
                for n in ["clk_p","clk_n","data0_p","data0_n","data1_p","data1_n","data2_p","data2_n","sda","scl","cec"]:
                    if hasattr(p, n): setattr(self, n, getattr(p, n))
                self.hdp = 1
        pads_w = PadsWrap(pads)

        self.submodules.videophy = ClockDomainsRenamer({"sys":"hdmi","sys5x":"hdmi5x"})(VideoGowinHDMIPHY(pads_w))
        self.submodules.vtg      = ClockDomainsRenamer("hdmi")(VideoTimingGenerator(default_video_timings="640x480@60Hz"))

        # Two dual-port line buffers (16b x 640)
        mem0 = Memory(16, HRES); mem1 = Memory(16, HRES)
        self.specials += mem0, mem1
        p0 = LinebufPorts(mem0); p1 = LinebufPorts(mem1)
        self.specials += p0.wr, p0.rd, p1.wr, p1.rd

        # ------------- Line handshake (HDMI -> SYS) -------------
        de_d       = Signal()
        req_tog_h  = Signal(reset=0)
        y_cap_h    = Signal(16)
        self.sync.hdmi += de_d.eq(self.vtg.source.de)
        new_line_h = Signal(); self.comb += new_line_h.eq(~de_d & self.vtg.source.de)
        self.sync.hdmi += If(new_line_h, y_cap_h.eq(self.vtg.source.vcount), req_tog_h.eq(~req_tog_h))

        # CDC to sys
        y_cap_s   = Signal(16)
        req_tog_s = Signal()
        self.specials += MultiReg(y_cap_h,   y_cap_s,   "sys")
        self.specials += MultiReg(req_tog_h, req_tog_s, "sys")

        # ------------- Buffer select ping-pong -------------
        buf_sel_h = Signal(reset=0)   # 0: HDMI reads mem0 this line; 1: reads mem1
        self.sync.hdmi += If(new_line_h, buf_sel_h.eq(~buf_sel_h))
        buf_sel_s = Signal()
        self.specials += MultiReg(buf_sel_h, buf_sel_s, "sys")

        # Bars writer (sys) writes the opposite buffer for the next line
        self.submodules.writer = BarsLineWriter(p0.wr, p1.wr)
        self.comb += [
            self.writer.req_y.eq(y_cap_s),
            self.writer.req_toggle.eq(req_tog_s),
            self.writer.buf_sel_sys.eq(buf_sel_s),
        ]

        # HDMI scanout
        self.submodules.scan = ClockDomainsRenamer("hdmi")(Scanout(p0.rd, p1.rd, self.vtg.source, self.videophy.sink, buf_sel_h))

        # ---------------- LEDs ----------------
        leds = platform.request_all("led_n")
        hb_sys  = Signal(26); self.sync     += hb_sys.eq(hb_sys + 1)
        hb_hdmi = Signal(24); self.sync.hdmi+= hb_hdmi.eq(hb_hdmi + 1)
        if len(leds) >= 2:
            self.comb += [leds[0].eq(~hb_sys[25]), leds[1].eq(~hb_hdmi[23])]

def main():
    from litex.build.parser import LiteXArgumentParser
    p = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = p.parse_args()
    soc  = Top()
    bld  = Builder(soc)
    if args.build: bld.build()
    if args.load:
        prog = soc.platform.create_programmer()
        try:
            bit = bld.get_bitstream_filename(mode="sram")
        except TypeError:
            bit = bld.get_bitstream_filename()
        prog.load_bitstream(bit)

if __name__ == "__main__":
    main()
