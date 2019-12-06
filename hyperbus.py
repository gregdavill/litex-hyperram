# This file is Copyright (c) 2019 Antti Lukats <antti.lukats@gmail.com>
# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# This file is Copyright (c) 2019 Gregory Davill <greg.davill@gmail.com>
# License: BSD

from migen import *

from litex.soc.interconnect import wishbone
#from migen.genlib.io import DDRInput, DDROutput

from litex.soc.cores.clock import *

from migen.genlib.cdc import MultiReg


# HyperRAM -----------------------------------------------------------------------------------------

class HyperRAM(Module):
    """HyperRAM

    Provides a standard HyperRAM core that works at 1:1 system clock speeds
    - PHY is device dependent for DDR IO primitives
      - ECP5 (done)
    - 90 deg phase shifted clock required from PLL
    - Burst R/W supported if bus is ready
    - Latency indepedent reads (RWDS strobing)

    This core favors performance over portability

    """
    def __init__(self, pads):
        self.pads = pads
        self.bus  = bus = wishbone.Interface()


        # # #

        clk_en    = Signal()
        cs        = Signal()
        ca       = Signal(48)
        sr_out        = Signal(48)
        sr_in        = Signal(32)
        sr_rwds = Signal(4)

        latency = Signal(4, reset=11)
        ca_load = Signal()
        dq_load = Signal()
        
        phy = HyperBusPHY(pads)
        self.submodules += phy

        self.comb += [
            phy.clk_en.eq(clk_en)
        ]

        # Drive rst_n, cs_n, clk from internal signals ---------------------------------------------
        if hasattr(pads, "rst_n"):
            self.comb += pads.rst_n.eq(1)
        self.comb += phy.cs.eq(~cs)
        
        # Data Out Shift Register (for write) -------------------------------------------------
        self.sync += [
            sr_out.eq(Cat(C(0,16), sr_out[:-16])),
            sr_rwds.eq(Cat(C(0,2), sr_rwds[:-2])),
            If(ca_load, 
                sr_out.eq(ca)
            ),
            If(dq_load,
                sr_out[:16].eq(0),
                sr_out[16:].eq(self.bus.dat_w),
                sr_rwds.eq(~self.bus.sel)
            )
        ]

        # Data in Shift Register
        dqi = Signal(16)
        self.sync += dqi.eq(phy.dq_in) # Store last sample, to align edges.
        self.sync += [
            If(phy.rwds_in == 0b01, # RAM indicates to us a valid word with RWDS strobes
                sr_in.eq(Cat(phy.dq_in[8:], dqi[:8], sr_in[:-16]))
            )
        ]

        self.comb += [
            bus.dat_r.eq(Cat(phy.dq_in[8:], dqi[:8], sr_in[:-16])), # To Wishbone
            phy.dq_out.eq(sr_out[-16:]),  # To HyperRAM
            phy.rwds_out.eq(sr_rwds[-2:]) # To HyperRAM
        ]

        # Command generation -----------------------------------------------------------------------
        self.comb += [
            ca[47].eq(~self.bus.we),          # R/W#
            ca[45].eq(1),                     # Burst Type (Linear)
            ca[16:35].eq(self.bus.adr[2:21]), # Row & Upper Column Address
            ca[1:3].eq(self.bus.adr[0:2]),    # Lower Column Address
            ca[0].eq(0),                      # Lower Column Address
        ]

        self.counter = counter = Signal(8)
        counter_rst = Signal()

        BURST_MAX = 5

        burst_count = Signal(max=(BURST_MAX+1))


        last_adr = Signal(32)

        # Create FSM to handle sequence, give better control over readouts
        self.submodules.fsm = fsm = FSM(reset_state='IDLE')
        

        self.sync += [
            # Cycle counter
            counter.eq(counter + 1),
            If(counter_rst,
                counter.eq(1)
            ),

            # Burst counter
            If(bus.ack,
                burst_count.eq(burst_count + 1)
            ),
            If(fsm.ongoing('IDLE'),
                burst_count.eq(0)
            )
        ]

        fsm.act('IDLE',
            If(bus.cyc & bus.stb, counter_rst.eq(1), 
                NextState('CMD'),
                NextValue(clk_en,1),
                NextValue(cs,1), 
                NextValue(phy.dq_oe,1), 
                ca_load.eq(1),
            ),
        )

        fsm.act('CMD',
            If(counter == 3, counter_rst.eq(1), 
                If(self.bus.we,
                    NextState('LATENCY_WRITE'),
                ).Else(
                    NextState('RWDS_WAIT'),
                ),
            )
        )

        fsm.act('RWDS_WAIT',
            NextValue(phy.dq_oe,0),
            If((phy.rwds_in == 0b01) & (counter > 3), # Skip over initial RWDS noise from CMD cycle
                counter_rst.eq(1), 
                NextState('READ_START'),
                
            )
        )

        fsm.act('READ_START',
            If(phy.rwds_in == 0b01, 
                
                NextValue(last_adr, bus.adr),
                counter_rst.eq(1), 
                NextState('READ_ACK'),
                #NextValue(cs,0), 
            )   
        )

        fsm.act('READ_ACK',
            If(phy.rwds_in == 0b01, 
                bus.ack.eq(1),
                NextValue(last_adr, bus.adr),
                counter_rst.eq(1), 
                NextState('READ_BURST'),
            )   
        )

        fsm.act('READ_BURST',
            If(bus.cyc & bus.stb & (bus.adr == (last_adr + 1)) & (burst_count < BURST_MAX),
                If(phy.rwds_in == 0b01,
                    counter_rst.eq(1), 
                    NextState('READ_ACK'),
                )
            ).Else(
                NextValue(clk_en,0), 
                NextValue(cs,0), 
                NextState('IDLE'),
            )
        )

        fsm.act('LATENCY_WRITE',
            NextValue(phy.dq_oe,0),
            If(counter == (latency-1), counter_rst.eq(1), 
                NextState('W_PREP'),
                NextValue(phy.dq_oe,1),
                NextValue(phy.rwds_oe,1),  
            )
        )

        fsm.act('W_PREP',
            dq_load.eq(1),
            bus.ack.eq(1),
            counter_rst.eq(1),
            NextState('WRITE_BURST'),
            NextValue(last_adr, bus.adr),
        ) 

        fsm.act('WRITE_BURST',
            If(bus.cyc & bus.stb & (bus.adr == (last_adr + 1)) & (burst_count < BURST_MAX),
                NextState('W_PREP'),
            ).Else(
                counter_rst.eq(1), NextState('WRITE_FINISH'),
            )
        )

        fsm.act('WRITE_FINISH',
            NextValue(clk_en,0), 
            counter_rst.eq(1), NextState('CLEANUP'),
        )

        fsm.act('CLEANUP',
            NextValue(cs,0), 
            counter_rst.eq(1), NextState('IDLE'),
            NextValue(phy.rwds_oe,0), 
            NextValue(phy.dq_oe,0),
        )


        self.dbg = [
            bus,
            phy.clk_en,
            phy.dq_in,
            phy.dq_out,
            phy.dq_oe,
            phy.rwds_oe,
            phy.rwds_in,
            phy.rwds_out,
            counter_rst,
            dqi,
            sr_in,
            sr_out,
            ca,
            counter,
            dq_load,
            last_adr,

        ]

class HyperBusPHY(Module):

    def add_tristate(self, pad):
        t = TSTriple(len(pad))
        self.specials += t.get_tristate(pad)
        return t

    def __init__(self, pads):
        
        # # #
        self.clk_en = Signal()
        self.cs = Signal()

        self.dq_oe = Signal()
        self.dq_in = Signal(16)
        self.dq_out = Signal(16)

        self.rwds_oe = Signal()
        self.rwds_in = Signal(2)
        self.rwds_out = Signal(2)

        dq        = self.add_tristate(pads.dq) if not hasattr(pads.dq, "oe") else pads.dq
        rwds      = self.add_tristate(pads.rwds) if not hasattr(pads.rwds, "oe") else pads.rwds


        # Shift non DDR signals to match the FF's inside DDR modules.
        self.specials += MultiReg(self.cs, pads.cs_n, n=2)

        self.specials += MultiReg(self.rwds_oe, rwds.oe, n=2)
        self.specials += MultiReg(self.dq_oe, dq.oe, n=2)
        
        # mask off clock when no CS
        clk_en = Signal()
        self.comb += clk_en.eq(self.clk_en & ~self.cs)
        
        # CLK
        if hasattr(pads, "clk"):
            self.specials += [
                Instance("ODDRX1F",
                    i_D0=0,
                    i_D1=clk_en,
                    i_SCLK=ClockSignal("sys_shift"),
                    i_RST=ResetSignal("sys_shift"),
                    o_Q=pads.clk
                )
            ]
        else: # DIFF PAIR CLK
            self.specials += [
                Instance("ODDRX1F",
                    i_D0=Cat(0,1),
                    i_D1=Cat(clk_en, ~clk_en),
                    i_SCLK=ClockSignal("sys_shift"),
                    i_RST=ResetSignal("sys_shift"),
                    o_Q=Cat(pads.clk_p,pads.clk_n)
                )
            ]

        # DQ_out
        for i in range(8):
            self.specials += [
                Instance("ODDRX1F",
                    i_D0=self.dq_out[8+i],
                    i_D1=self.dq_out[i],
                    i_SCLK=ClockSignal("sys"),
                    i_RST=ResetSignal("sys"),
                    o_Q=dq.o[i]
                )
            ]
        

        # DQ_in
        for i in range(8):
            self.specials += [
                Instance("IDDRX1F",
                    i_D=dq.i[i],
                    i_SCLK=ClockSignal("sys"),
                    i_RST=ResetSignal("sys"),
                    o_Q0=self.dq_in[i+8],
                    o_Q1=self.dq_in[i]
                )
            ]
        
        # RWDS_out
        self.specials += [
            Instance("ODDRX1F",
                i_D0=self.rwds_out[1],
                i_D1=self.rwds_out[0],
                i_SCLK=ClockSignal("sys"),
                i_RST=ResetSignal("sys"),
                o_Q=rwds.o
            )
        ]

        # RWDS_in
        self.specials += [
            Instance("IDDRX1F",
                i_D=rwds.i,
                i_SCLK=ClockSignal("sys"),
                i_RST=ResetSignal("sys"),
                o_Q0=self.rwds_in[1],
                o_Q1=self.rwds_in[0]
            )
        ]
        

        
