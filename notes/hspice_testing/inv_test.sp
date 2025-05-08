
.include "/proj/cad/library/mosis/GF65_LPe/cmos10lpe_CDS_oa_dl064_11_20160415/models/YI-SM00030/Hspice/models/design.inc"
.include "inv.pex.sp"

.option post runlvl=5

xi GND! OUT VDD! IN inv
vdd VDD! GND! 1.2v
vin IN GND! pwl(0ns 1.2v 1ns 1.2v 1.05ns 0v 6ns 0v 6.05ns 1.2v 12ns 1.2v)
cout OUT GND! 15f

.tr 100ps 12ns

.measure tran trise trig v(IN) val=0.6v fall=1 targ v(OUT) val=0.6v rise=1 $measure tlh at 0.6v
.measure tran tfall trig v(IN) val=0.6v rise=1 targ v(OUT) val=0.6v fall=1 $measure tpl at 0.6v

.measure tavg param = '(trise+tfall)/2' 
.measure tdiff param='abs(trise-tfall)' 
.measure delay param='max(trise,tfall)' $

.measure tran iavg avg i(vdd) from=0 to=10n 
.measure energy param='1.2*iavg*10n' 
.measure edp1 param='abs(delay*energy)'

.end

