$ HSPICE Testbench for XNOR2 Gate
$ Include transistor model and XNOR2 layout-extracted netlist
.include "/proj/cad/library/mosis/GF65_LPe/cmos10lpe_CDS_oa_dl064_11_20160415/models/YI-SM00030/Hspice/models/design.inc"
.include "/home/010/f/fx/fxp200004/cad/gf65/tutorial/xnor2/xnor2_plex/xnor2.pex.sp"

.option post runlvl=5

$ Optional parameter sweep (if parameter k or others are defined in XNOR2)
.param k=1

$ XNOR2 instance: xnor2 Gate definition
xi OUT GND! VDD! A B XNOR2

$ Power supply
vdd VDD! GND! 1.2v

$ Input waveforms (30 ps slew from 0.2*VDD to 0.8*VDD ≈ 0.24V to 0.96V)
vin1 A GND! pwl(0ns 0v 1ns 0v 1.05ns 1.2v 4ns 1.2v 4.05ns 0v 5ns 0v 5.05ns 1.2V 11ns 1.2v 11.05ns 0V)
vin2 B GND! pwl(0ns 0v 1ns 0v 1.05ns 1.2v 6ns 1.2v 6.05ns 0v 8ns 0 8.05ns 1.2v 11ns 1.2v 11.05ns 0v)

$ Load capacitance = 30 fF
cload OUT GND! 30f

$ Transient analysis with parameter sweep
.tr 100ps 12ns sweep k 0.5 5 0.1

$ Delay measurements
.measure tran trise trig v(A) val=0.6v fall=1 targ v(OUT) val=0.6v rise=1
.measure tran tfall trig v(A) val=0.6v rise=1 targ v(OUT) val=0.6v fall=1
.measure tavg param = '(trise+tfall)/2'
.measure tdiff param='abs(trise-tfall)'
.measure delay param='max(trise,tfall)'

$ Energy and power analysis (method 1)
.measure tran iavg avg i(vdd) from=0n to=10n
.measure energy param='1.2*iavg*10n'
.measure edp1 param='abs(delay*energy)'
.measure aedp1 param='edp1*1.107e7'

$ Energy and power analysis (method 2)
.measure tran t1 when v(A)=0.01 rise=1
.measure tran t2 when v(B)=0.01 rise=1
.measure tran t3 when v(OUT)=0.01 fall=1
.measure tran t4 when v(A)=1.19 fall=1
.measure tran t5 when v(OUT)=1.19 rise=1
.measure tran t6 when v(A)=1.19 rise=1
.measure tran t7 when v(B)=0.01 fall=1
.measure tran t8 when v(A)=0.01 fall=1
.measure tran i1 avg i(vdd) from=t1 to=t3
.measure tran i2 avg i(vdd) from=t4 to=t5
.measure energy1 param='1.2*i1*(t3-t1)'
.measure energy2 param='1.2*i2*(t5-t4)'
.measure energysum param='energy1+energy2'
.measure edp2 param='abs(delay*energysum)'
.measure aedp2 param = 'edp2*1.107e7'

.end
