$ HSPICE Testbench for D Flip-Flop with Reset
$ Include transistor model and layout-extracted netlist
.include "/proj/cad/library/mosis/GF65_LPe/cmos10lpe_CDS_oa_dl064_11_20160415/models/YI-SM00030/Hspice/models/design.inc"
.include "/home/010/f/fx/fxp200004/cad/gf65/tutorial/dff/lvs/dff.pex.sp"

.option post runlvl=5

$ DFF instance: xi GND! Q VDD! CLK R D
xi GND! Q VDD! CLK R D dff

$ Power supply
vdd VDD! GND! 1.2v

$ Input waveforms (30 ps slew from 0.2*VDD to 0.8*VDD ≈ 0.24V to 0.96V)
$ Uniform clock signal with 5ns period (50% duty cycle)
vclk CLK GND! PULSE(0    1.2    1.5ns    50ps   50ps    1.5ns   3ns)

$ Reset pulse signal (from 0 to 1.2V at 2ns and back to 0V at 8ns)
vin_r R GND! pwl(0ns 0v 3ns 0v 3.05ns 1.2v 5ns 1.2v 5.05ns 0v 8.45ns 0v 8.5ns 1.2v 10ns 1.2v 10.05ns 0v)

$ D input signal (Toggle between 0V and 1.2V, 3ns interval)
vin_d D GND! pwl(0ns 0v 1ns 0v 1.05ns 1.2v 3.5ns 1.2v 3.55ns 0v 6ns 0v 6.05ns 1.2v 9ns 1.2v 9.05ns 0v)

$ Load capacitance: 30fF
cload Q GND! 30f

$ Initial condition: Q starts at 0
.ic V(Q)=0.0

$ Transient analysis: Run from 0ns to 10ns
.tran 1ps 10ns

$ Timing measurement: Delay from clock to Q
.measure tran tclktoq1 TRIG v(CLK) VAL=0.6 FALL=1 TARG v(Q) VAL=0.6 RISE=1
.measure tran tclktoq2 TRIG v(CLK) VAL=0.6 FALL=2 TARG v(Q) VAL=0.6 FALL=1

$ Reset behavior: Test if Q resets to 0 during reset pulse
.measure tran treset TRIG v(R) VAL=1.0 FALL=1 TARG v(Q) VAL=0.6 RISE=1

$ Energy and power analysis
.measure tran iavg avg i(vdd) from=0n to=10n
.measure energy param='1.2*iavg*10n'
.measure edp param='abs(tclktoq1*energy)'
.measure aedp param='edp*1.107e7'

.end

