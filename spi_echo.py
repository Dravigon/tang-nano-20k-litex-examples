#!/usr/bin/env python3
# Tang Nano 20K — Simple SPI Echo Debug (no protocol, just verify SPI works)
# Should echo the PREVIOUS byte received (first byte after CS↓ is always 0xA5)

from migen import *
from litex.gen import LiteXModule
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock.gowin_gw2a import GW2APLL

# Prefer local platform module if present
try:
    from platforms import sipeed_tang_nano_20k
except Exception:
    from litex_boards.platforms import sipeed_tang_nano_20k

# -------- CRG --------
class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq=int(48e6)):
        self.cd_sys = ClockDomain()
        pll = GW2APLL(device=platform.device, devicename=platform.devicename)
        self.submodules.pll = pll
        pll.register_clkin(platform.request("clk27"), 27e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# --- Simple SPI Slave Echo (MODE0) ---
class SimpleSPIEcho(LiteXModule):
    """
    Minimal SPI Slave (MODE0, MSB-first).
    First byte after CS↓ returns 0xA5.
    Subsequent bytes echo the previous byte received.
    """
    def __init__(self, pads):
        # Normalize pad names
        sck_i  = pads.clk if hasattr(pads, "clk") else pads.sck
        mosi_i = pads.mosi
        miso_o = pads.miso
        csn_i  = pads.cs_n

        # Debug counters
        self.byte_count = byte_count = Signal(16)
        self.last_rx = last_rx = Signal(8)
        self.last_tx = last_tx = Signal(8)
        
        # Sync signals into sys domain (double register for metastability)
        sck_m = Signal(); sck = Signal(); sck_d = Signal()
        mosi_m = Signal(); mosi = Signal()
        cs_m = Signal(reset=1); cs = Signal(reset=1); cs_d = Signal(reset=1)

        self.sync += [
            sck_m.eq(sck_i),   sck.eq(sck_m),   sck_d.eq(sck),
            mosi_m.eq(mosi_i), mosi.eq(mosi_m),
            cs_m.eq(csn_i),    cs.eq(cs_m),     cs_d.eq(cs),
        ]

        # Edge detection
        cs_active = ~cs
        sck_rise = ~sck_d & sck
        sck_fall = sck_d & ~sck
        cs_fall = cs_d & ~cs
        cs_rise = ~cs_d & cs

        # SPI state
        bitcnt = Signal(3)
        rx_shift = Signal(8)
        tx_shift = Signal(8)
        echo_byte = Signal(8, reset=0xA5)  # What to echo next
        miso_q = Signal(reset=1)

        # Drive MISO (high when inactive)
        self.comb += miso_o.eq(Mux(cs_active, miso_q, 1))

        # Main SPI logic
        self.sync += [
            # CS falling edge: start new transaction
            If(cs_fall,
                bitcnt.eq(0),
                rx_shift.eq(0),
                tx_shift.eq(0xA5),          # First byte is always 0xA5
                miso_q.eq(1),               # MSB of 0xA5
                echo_byte.eq(0xA5),         # Reset echo
                byte_count.eq(0),           # Reset counter
            ),
            
            # CS rising edge: end transaction
            If(cs_rise,
                bitcnt.eq(0),
            ),
            
            # While CS is active
            If(cs_active,
                # Sample MOSI on SCK rising edge
                If(sck_rise,
                    rx_shift.eq(Cat(mosi, rx_shift[:-1])),  # Shift in MSB first
                    
                    # Byte complete?
                    If(bitcnt == 7,
                        bitcnt.eq(0),
                        echo_byte.eq(Cat(mosi, rx_shift[:-1])),  # Save for next byte
                        last_rx.eq(Cat(mosi, rx_shift[:-1])),    # Debug
                        byte_count.eq(byte_count + 1),
                    ).Else(
                        bitcnt.eq(bitcnt + 1),
                    )
                ),
                
                # Update MISO on SCK falling edge
                If(sck_fall,
                    # Start of new byte?
                    If(bitcnt == 0,
                        tx_shift.eq(echo_byte),
                        miso_q.eq(echo_byte[7]),
                        last_tx.eq(echo_byte),  # Debug
                    ).Else(
                        # Shift left and output MSB
                        tx_shift.eq(Cat(Constant(0,1), tx_shift[:-1])),
                        miso_q.eq(tx_shift[6]),  # Next bit after shift
                    )
                )
            ).Else(
                miso_q.eq(1),  # Idle high
            )
        ]

# -------- Top SoC --------
class Top(SoCCore):
    def __init__(self, sys_clk_freq=int(48e6)):
        platform = sipeed_tang_nano_20k.Platform()
        self.submodules.crg = _CRG(platform, sys_clk_freq=sys_clk_freq)
        
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0x0,
                         integrated_main_ram_size=0x1000)
        
        # SPI pads
        try:
            pads = platform.request("custom_spi", 0)
        except Exception:
            pads = platform.request("spi")
            
        self.submodules.spi = SimpleSPIEcho(pads)
        
        # Debug LEDs (active low on Tang Nano 20K)
        try:
            led0 = platform.request("led_n", 0)
            led1 = platform.request("led_n", 1)
            led2 = platform.request("led_n", 2)
            led3 = platform.request("led_n", 3)
            led4 = platform.request("led_n", 4)
            led5 = platform.request("led_n", 5)
            
            # LED0: CS active
            self.comb += led0.eq(pads.cs_n)
            
            # LED1: Blink on each byte
            byte_pulse = Signal()
            byte_cnt_d = Signal(16)
            self.sync += [
                byte_cnt_d.eq(self.spi.byte_count),
                byte_pulse.eq(self.spi.byte_count != byte_cnt_d)
            ]
            pulse_stretch = Signal(20)
            self.sync += If(byte_pulse, pulse_stretch.eq((1<<20)-1)
                        ).Elif(pulse_stretch != 0, pulse_stretch.eq(pulse_stretch-1))
            self.comb += led1.eq(~(pulse_stretch != 0))
            
            # LED2-5: Show last received byte (low nibble)
            self.comb += [
                led2.eq(~self.spi.last_rx[0]),
                led3.eq(~self.spi.last_rx[1]),
                led4.eq(~self.spi.last_rx[2]),
                led5.eq(~self.spi.last_rx[3]),
            ]
        except Exception:
            pass

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=sipeed_tang_nano_20k.Platform)
    args = parser.parse_args()

    soc = Top()
    bld = Builder(soc)
    if args.build:
        bld.build()
    if args.load:
        soc.platform.create_programmer().load_bitstream(bld.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()