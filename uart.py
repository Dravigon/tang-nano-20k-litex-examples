#!/usr/bin/env python3
from migen import *
from litex.gen import *
from platforms import sipeed_tang_nano_20k
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder
from litex.soc.cores.uart import UARTPHY, UART

# Minimal CRG: use 27 MHz directly
class _CRG(LiteXModule):
    def __init__(self, platform):
        self.cd_sys = ClockDomain()
        clk27 = platform.request("clk27")
        self.comb += self.cd_sys.clk.eq(clk27)


class UARTBlink(LiteXModule):
    def __init__(self, pads, led_n_byte, led_n_heartbeat, sys_clk_freq):
        from litex.soc.cores.uart import UARTPHY, UART

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
        self.comb += byte_pulse.eq(self.uart.source.valid & ~prev_valid)
        self.sync += prev_valid.eq(self.uart.source.valid)

        self.sync += [
            If(rx_edge | byte_pulse,           # any RX activity OR a good byte
                flash_cnt.eq(int(27e6*0.05)),
                flashing.eq(1)
            ).Elif(flashing,
                If(flash_cnt != 0,
                    flash_cnt.eq(flash_cnt - 1)
                ).Else(
                    flashing.eq(0)
                )
            )
        ]
        self.comb += led_n_byte.eq(~flashing)  # active-low LED

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


class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=int(27e6), **kwargs):
        platform = sipeed_tang_nano_20k.Platform()
        SoCCore.__init__(self, platform, sys_clk_freq,
                         cpu_type=None,
                         integrated_rom_size=0,
                         with_uart=False,
                         **kwargs)

        # Clocking
        self.crg = _CRG(platform)

        # PADS
        uart_pads = platform.request("custom_serial")  # your mapping: TX=48, RX=51
        led_byte  = platform.request("led_n", 0)       # flashes on byte
        led_hb    = platform.request("led_n", 1)       # heartbeat

        # Logic
        self.submodules.uart_blink = UARTBlink(uart_pads, led_byte, led_hb, sys_clk_freq)

def main():
    soc = BaseSoC()
    builder = Builder(soc)
    builder.build()
    soc.platform.create_programmer().load_bitstream(
        builder.get_bitstream_filename(mode="sram")
    )

if __name__ == "__main__":
    main()
