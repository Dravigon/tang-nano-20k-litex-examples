#!/usr/bin/env python3
# Tang Nano 20K — SPI (MODE0) framebuffer RX with sticky ACKs (CS stays LOW)
#
# Protocol from ESP32-S3 master:
#   SOF:  0xAA 0x44
#   ROW:  0xAA 0x55 'R' y_lo y_hi + 1280 bytes (RGB565 LE, 640 px)
#         <- MISO: 'K' (0x4B) sticky while CS LOW after row stored
#   EOF:  0xAA 0x66
#         <- MISO: 'F' (0x46) sticky while CS LOW
#
# NOTE: This variant assumes CS remains LOW for the whole transfer. Because of that,
# we can switch the TX byte source exactly at byte boundaries (no mid-byte alignment
# or "forced" path needed).

from migen import *
from litex.gen import LiteXModule
from litex.soc.interconnect import stream
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from platforms import sipeed_tang_nano_20k
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

# -------------------- Clock Reset Gen --------------------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(108e6)):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# -------------------- SPI MODE0 slave (no "force" path) --------------------
# - CPOL=0, CPHA=0: sample MOSI on SCK↑, update MISO on SCK↓
# - MSB-first
# - Double-flop sync for SCK/MOSI/CS
# - Exposes rx_stb/rx_data (one pulse per received byte)
# - TX path reloads at *byte boundaries*:
#     * When ack_en=0  -> keep reloading IDLE byte (0xFF)
#     * When ack_en=1  -> keep reloading ack_byte (0x4B or 0x46)
class SPISlaveMode0(LiteXModule):
    def __init__(self, pads, idle_tx=0xFF):
        sck_i  = pads.clk if hasattr(pads, "clk") else pads.sck
        mosi_i = pads.mosi
        miso_o = pads.miso
        csn_i  = pads.cs_n   # Tang Nano custom_spi uses active-low CS

        # 2-FF sync to sysclk
        sck_m, sck, sck_d = Signal(), Signal(), Signal()
        mosi_m, mosi      = Signal(), Signal()
        cs_m, cs, cs_d    = Signal(reset=1), Signal(reset=1), Signal(reset=1)
        self.sync += [
            sck_m.eq(sck_i),  sck.eq(sck_m),  sck_d.eq(sck),
            mosi_m.eq(mosi_i), mosi.eq(mosi_m),
            cs_m.eq(csn_i),   cs.eq(cs_m),    cs_d.eq(cs),
        ]
        cs_active = ~cs
        sck_rise  = ~sck_d &  sck
        sck_fall  =  sck_d & ~sck
        cs_fall   =  cs_d  & ~cs
        cs_rise   = ~cs_d  &  cs

        # RX assembler
        self.rx_stb  = Signal()
        self.rx_data = Signal(8)
        rx_shift = Signal(8)
        rx_bit   = Signal(3)

        # TX shifter (MSB-first) + byte counter to detect boundaries
        tx_shift = Signal(8, reset=idle_tx)
        tx_bit   = Signal(3, reset=7)
        miso_q   = Signal(reset=1)

        # ACK control from parser
        self.ack_en   = Signal(reset=0)      # 1 = stream ACK byte on every next byte
        self.ack_byte = Signal(8, reset=0xFF)

        # connect to pad (idle-high when CS is HIGH)
        self.comb += miso_o.eq(Mux(cs_active, miso_q, 1))
        self.cs_active = Signal()
        self.comb += self.cs_active.eq(cs_active)

        # defaults
        self.sync += self.rx_stb.eq(0)

        # On CS edges
        self.sync += If(cs_fall,
            rx_bit.eq(0),
            rx_shift.eq(0),
            tx_bit.eq(7),
            # preload the current source (idle or ack) at start of transaction
            If(self.ack_en,
                tx_shift.eq(self.ack_byte),
                miso_q.eq(self.ack_byte[7])
            ).Else(
                tx_shift.eq(idle_tx),
                miso_q.eq((idle_tx >> 7) & 1)
            )
        ).Elif(cs_rise,
            rx_bit.eq(0),
            tx_bit.eq(7),
        )

        # In transaction
        self.sync += If(cs_active,
            # RX on rising edges (MSB-first)
            If(sck_rise,
                rx_shift.eq(Cat(mosi, rx_shift[:-1])),
                If(rx_bit == 7,
                    self.rx_data.eq(Cat(mosi, rx_shift[:-1])),
                    self.rx_stb.eq(1),
                    rx_bit.eq(0)
                ).Else(
                    rx_bit.eq(rx_bit + 1)
                )
            ),

            # TX on falling edges (MSB-first)
            If(sck_fall,
                miso_q.eq(tx_shift[7]),
                # At byte boundary (after 8 bits shifted), reload from the selected source
                If(tx_bit == 0,
                    tx_bit.eq(7),
                    If(self.ack_en,
                        tx_shift.eq(self.ack_byte)
                    ).Else(
                        tx_shift.eq(idle_tx)
                    )
                ).Else(
                    tx_bit.eq(tx_bit - 1),
                    tx_shift.eq(Cat(0, tx_shift[:-1]))   # left shift: MSB-first
                )
            )
        ).Else(
            miso_q.eq(1),
            self.rx_stb.eq(0)
        )

# -------------------- Parser -> Row Stream (RGB565 LE) --------------------
# - Runs a byte-level FSM on rx_stb/rx_data
# - As soon as the last row byte is received, sets ack_en=1, ack_byte=0x4B
#   so the next byte (with CS still LOW) comes out as 'K' and repeats forever
#   until the next header is parsed.
# - On EOF (AA 66) sets ack_byte=0x46 similarly.
class SPIRxFramebuffer(LiteXModule):
    def __init__(self, pads, hres=640, vres=480, autodrain=True):
        self.hres, self.vres = hres, vres
        self.submodules.spi  = SPISlaveMode0(pads, idle_tx=0xFF)

        # Row stream endpoint
        row_desc = stream.EndpointDescription(
            payload_layout=[("data", 16), ("y", 10)],
            param_layout=[("sof", 1), ("eol", 1)],
        )
        self.row_source = stream.Endpoint(row_desc)
        if autodrain:
            self.comb += self.row_source.ready.eq(1)

        rx_stb  = self.spi.rx_stb
        rx_data = self.spi.rx_data

        byte_cnt = Signal(16)     # [0..1280]
        y_lo     = Signal(8)
        row_y    = Signal(10)
        pix_lo   = Signal(8)
        have_lo  = Signal()

        # Drive ACK defaults
        self.comb += [
            self.spi.ack_en.eq(0),
            self.spi.ack_byte.eq(0xFF),
        ]

        fsm = FSM(reset_state="WAIT_SOF_AA")
        self.submodules += fsm

        # SOF: AA 44
        fsm.act("WAIT_SOF_AA",
            NextValue(byte_cnt, 0),
            NextValue(have_lo, 0),
            If(rx_stb & (rx_data == 0xAA), NextState("WAIT_SOF_44"))
        )
        fsm.act("WAIT_SOF_44",
            If(rx_stb,
                If(rx_data == 0x44, NextState("IN_FRAME"))
                .Else(NextState("WAIT_SOF_AA"))
            )
        )

        # IN_FRAME: rows or EOF
        fsm.act("IN_FRAME",
            If(rx_stb,
                If(rx_data == 0xAA, NextState("ROW_HDR_55"))   # next row
                .Elif(rx_data == 0x66, NextState("EOF_ACK"))  # EOF -> sticky F
            )
        )

        # ROW header: AA 55 'R' y_lo y_hi
        fsm.act("ROW_HDR_55",
            If(rx_stb,
                If(rx_data == 0x55, NextState("ROW_HDR_R"))
                .Else(NextState("WAIT_SOF_AA"))
            )
        )
        fsm.act("ROW_HDR_R",
            If(rx_stb,
                If(rx_data == ord('R'), NextState("ROW_Y_LO"))
                .Else(NextState("WAIT_SOF_AA"))
            )
        )
        fsm.act("ROW_Y_LO",
            If(rx_stb, NextValue(y_lo, rx_data), NextState("ROW_Y_HI"))
        )
        fsm.act("ROW_Y_HI",
            If(rx_stb,
                NextValue(row_y, Cat(y_lo, rx_data[:2])),
                NextValue(byte_cnt, 0),
                NextValue(have_lo, 0),
                NextState("ROW_PAYLOAD")
            )
        )

        # 1280B payload -> 640 pixels; after last byte, switch TX to 'K'
        fsm.act("ROW_PAYLOAD",
            self.row_source.valid.eq(0),
            If(rx_stb,
                If(~have_lo,
                    NextValue(pix_lo, rx_data),
                    NextValue(have_lo, 1)
                ).Else(
                    If(self.row_source.ready,
                        self.row_source.valid.eq(1),
                        self.row_source.data.eq(Cat(pix_lo, rx_data)),
                        self.row_source.y.eq(row_y),
                        self.row_source.sof.eq(byte_cnt == 0),
                        self.row_source.eol.eq(byte_cnt == (self.hres*2 - 2)),
                        NextValue(byte_cnt, byte_cnt + 2),
                        NextValue(have_lo, 0)
                    )
                )
            ),
            If(byte_cnt == (self.hres*2),
                NextState("ROW_ACK_K")
            )
        )

        # Sticky 'K' until next header (AA ...)
        fsm.act("ROW_ACK_K",
            self.spi.ack_en.eq(1),
            self.spi.ack_byte.eq(0x4B),
            If(rx_stb & (rx_data == 0xAA),
                NextState("ROW_ACK_SEEN_AA")
            )
        )
        fsm.act("ROW_ACK_SEEN_AA",
            self.spi.ack_en.eq(1),
            self.spi.ack_byte.eq(0x4B),
            If(rx_stb,
                If(rx_data == 0x55, NextState("ROW_HDR_R"))   # next row continues
                .Elif(rx_data == 0x66, NextState("EOF_ACK"))  # EOF follows
                .Else(NextState("WAIT_SOF_AA"))
            )
        )

        # Sticky 'F' until next SOF (AA 44)
        fsm.act("EOF_ACK",
            self.spi.ack_en.eq(1),
            self.spi.ack_byte.eq(0x46),
            If(rx_stb & (rx_data == 0xAA),
                NextState("EOF_ACK_WAIT_44")
            )
        )
        fsm.act("EOF_ACK_WAIT_44",
            self.spi.ack_en.eq(1),
            self.spi.ack_byte.eq(0x46),
            If(rx_stb,
                If(rx_data == 0x44, NextState("IN_FRAME"))
                .Else(NextState("WAIT_SOF_AA"))
            )
        )

# -------------------- SoC shell --------------------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(108e6), hres=640, vres=480):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(platform, sys_clk_freq=sys_clk_freq)

        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0x0,
                         integrated_main_ram_size=0x1000)

        pads = platform.request("custom_spi", 0)
        self.submodules.spi_fb = SPIRxFramebuffer(pads, hres=hres, vres=vres, autodrain=True)

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    parser.add_target_argument("--sys-clk-freq", default=108e6, type=float)
    parser.add_target_argument("--hres", default=640, type=int)
    parser.add_target_argument("--vres", default=480, type=int)
    args = parser.parse_args()

    soc = Top(sys_clk_freq=int(float(args.sys_clk_freq)),
              hres=int(args.hres), vres=int(args.vres))
    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)
    if args.load:
        soc.platform.create_programmer().load_bitstream(
            builder.get_bitstream_filename(mode="sram")
        )

if __name__ == "__main__":
    main()
