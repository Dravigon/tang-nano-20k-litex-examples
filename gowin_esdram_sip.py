# Minimal wrapper for Gowin Embedded SDR SDRAM (SIP) on Tang Nano 20K.
# Bus is 32-bit. 'length' is in 32-bit beats. Active-low cmd strobes handled for you.

from migen import *
from litex.gen import *

class GowinESDRAM_SIP(Module):
    def __init__(self, sdrc_clk, sdram_clk, rst_active_low=True):
        # -------- user interface (simple, ready/valid-ish) --------
        self.init_done  = Signal()   # 1 = SDRAM initialized
        self.busy       = Signal()   # 1 = controller busy (not ready for new cmd)

        # Command
        self.addr       = Signal(21)     # 21-bit address as expected by core
        self.length     = Signal(8)      # number of 32-bit beats (1..page)
        self.rd_req     = Signal()       # pulse 1 cycle to start read
        self.wr_req     = Signal()       # pulse 1 cycle to start write

        # Write channel
        self.wr_data        = Signal(32)
        self.wr_data_valid  = Signal()
        self.wr_data_ack    = Signal()   # core acks write data beats
        self.wstrb          = Signal(4)  # optional byte enables (1=write byte)

        # Read channel
        self.rd_data    = Signal(32)
        self.rd_valid   = Signal()

        # -------- internal wires for the "SDRAM pins" (not routed to IO) --------
        O_sdram_clk   = Signal()
        O_sdram_cke   = Signal()
        O_sdram_cs_n  = Signal()
        O_sdram_cas_n = Signal()
        O_sdram_ras_n = Signal()
        O_sdram_wen_n = Signal()
        O_sdram_dqm   = Signal(4)
        O_sdram_addr  = Signal(11)
        O_sdram_ba    = Signal(2)
        IO_sdram_dq   = TSTriple(32)
        self.specials += IO_sdram_dq.get_tristate(Signal(32))  # not externally used

        busy_n = Signal()

        # Reset polarity (core expects active-low)
        rst_n = ~ResetSignal() if not rst_active_low else ~ResetSignal()

        # If you don't care about byte-masking, write all bytes:
        dqm_in = ~self.wstrb    # 1 bit of DQM masks that byte. wstrb=1111 -> DQM=0000
        self.comb += If(self.wstrb == 0, dqm_in.eq(0x0))  # default to write-all if unused

        # --- Instantiate your generated SIP core ---
        self.specials += Instance("SDRAM_controller_top_SIP",
            # SDRAM side (tied to internal nets for embedded SDR)
            o_O_sdram_clk   = O_sdram_clk,
            o_O_sdram_cke   = O_sdram_cke,
            o_O_sdram_cs_n  = O_sdram_cs_n,
            o_O_sdram_cas_n = O_sdram_cas_n,
            o_O_sdram_ras_n = O_sdram_ras_n,
            o_O_sdram_wen_n = O_sdram_wen_n,
            o_O_sdram_dqm   = O_sdram_dqm,
            o_O_sdram_addr  = O_sdram_addr,
            o_O_sdram_ba    = O_sdram_ba,
            io_IO_sdram_dq  = IO_sdram_dq.io,

            # Clocks / power features
            i_I_sdrc_rst_n       = rst_n,
            i_I_sdrc_clk         = sdrc_clk,   # user/controller clock
            i_I_sdram_clk        = sdram_clk,  # SDRAM working clock (same freq; typically phase-shifted)
            i_I_sdrc_selfrefresh = 0,
            i_I_sdrc_power_down  = 0,

            # User command & data
            i_I_sdrc_wr_n          = ~self.wr_req,   # active-low
            i_I_sdrc_rd_n          = ~self.rd_req,   # active-low
            i_I_sdrc_addr          = self.addr,
            i_I_sdrc_data_len      = self.length,    # beats (32-bit)
            i_I_sdrc_dqm           = dqm_in,         # 1=masks that byte
            i_I_sdrc_data          = self.wr_data,

            o_O_sdrc_data          = self.rd_data,
            o_O_sdrc_init_done     = self.init_done,
            o_O_sdrc_busy_n        = busy_n,
            o_O_sdrc_rd_valid      = self.rd_valid,
            o_O_sdrc_wrd_ack       = self.wr_data_ack
        )

        self.comb += self.busy.eq(~busy_n)
