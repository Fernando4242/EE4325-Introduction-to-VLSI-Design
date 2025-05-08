$example HSPICE setup file

$transistor model
.include "/proj/cad/library/mosis/GF65_LPe/cmos10lpe_CDS_oa_dl064_11_20160415/models/YI-SM00030/Hspice/models/design.inc"
.include "nand2.pex.sp"

$.param Wn=2.52e-06
$.param k=1
.option post runlvl=5

$xi GND! OUT VDD! A B nand2
xi OUT GND! VDD! A B nand2

vdd VDD! GND! 1.2v
vin1 A GND! pwl(0ns 0v 1ns 0v 1.05ns 1.2v 4ns 1.2v 4.05ns 0v 5ns 0v 5.05ns 1.2V 11ns 1.2v 11.05ns 0V)
vin2 B GND! pwl(0ns 0v 1ns 0v 1.05ns 1.2v 6ns 1.2v 6.05ns 0v 8ns 0 8.05ns 1.2v 11ns 1.2v 11.05ns 0v)
cout OUT GND! 30f

$transient analysis
.tr 100ps 12ns $sweep k 0.5 5 0.1
$example of parameter sweep, replace numeric value W of pfet with WP in invlvs.sp
$.tr 100ps 12ns sweep WP 1u 9u 0.5u

.measure tran trise trig v(A) val=0.6v fall=1 targ v(OUT) val=0.6v rise=1 $measure tlh at 0.6v
.measure tran tfall trig v(A) val=0.6v rise=1 targ v(OUT) val=0.6v fall=1 $measure tpl at 0.6v
.measure tavg param = '(trise+tfall)/2' $calculate average delay for input a
.measure tdiff param='abs(trise-tfall)' $calculate delay difference for input a
.measure delay param='max(trise,tfall)' $calculate worst case delay for input a

$ method 1
.measure tran iavg avg i(vdd) from=0n to=10n $average current in one clock cycle
.measure energy param='1.2*iavg*10n' $calculate energy in one clock cycle
.measure edp1 param='abs(delay*energy)'
.measure aedp1 param='edp1*1.107e7'

$ method 2
.measure tran t1 when v(A)=0.01 rise=1
.measure tran t2 when v(B)=0.01 rise=1
.measure tran t3 when v(OUT)=0.01 fall=1
.measure tran t4 when v(A)=1.19 fall=1
.measure tran t5 when v(OUT)=1.19 rise=1
.measure tran t6 when v(A)=1.19 rise=1
.measure tran t7 when v(B)=0.01 fall=1
.measure tran t8 when v(A)=0.01 fall=1
.measure tran i1 avg i(vdd) from=t1 to=t3 $average current when output rise
.measure tran i2 avg i(vdd) from=t4 to=t5 $average current when output fall
.measure energy1 param='1.2*i1*(t3-t1)' $calculate energy when output rise
.measure energy2 param='1.2*i2*(t5-t4)' $calculate energy when output fall
.measure energysum param='energy1+energy2'
.measure edp2 param='abs(delay*energysum)'
.measure aedp2 param = 'edp2*1.107e7'

.end
