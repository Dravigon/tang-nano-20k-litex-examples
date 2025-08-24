# uart_hello.py
from migen import *
from litex.gen import *
from litex.soc.interconnect import stream

class UartHello(LiteXModule):
    def __init__(self, sink: stream.Endpoint, text="Hello, UART!\r\n", start_hold_cycles=1024):
        # Convert message to bytes
        msg = bytes(text, "ascii")

        # ROM for the message
        rom = Memory(8, len(msg), init=list(msg))
        rom_port = rom.get_port()
        self.specials += rom, rom_port

        period = Signal(25)  # at 27 MHz, bit 24 ~0.62s
        idx = Signal(max=len(msg)+1)
        sending   = Signal(reset=0)
        valid_reg = Signal(reset=0)
        data_reg  = Signal(8, reset=0)
        
        self.comb += [
            sink.valid.eq(valid_reg),
            sink.data.eq(data_reg),
            rom_port.adr.eq(idx),
        ]
        
        self.sync += [
            period.eq(period + 1),
        
            If(~sending & period[24],        # wait ~0.6s then (re)start
                sending.eq(1),
                idx.eq(0),
                valid_reg.eq(0)
            ).Elif(sending,
                If(idx < len(msg),
                    data_reg.eq(rom_port.dat_r),
                    valid_reg.eq(1),
                    If(sink.ready,
                        idx.eq(idx + 1)
                    )
                ).Else(
                    valid_reg.eq(0),
                    sending.eq(0)            # done; wait for next period
                )
            )
        ]
        