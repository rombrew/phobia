#!/home/amaora/util/gp
# vi: ft=conf

load 0 0 float 40 "/tmp/pm-PWM"

group 0 0
deflabel 0 "(us)"

page "VSI Output"
label 1 "(V)"
figure 0 1 "m.vsi[A]"
figure 0 2 "m.vsi[B]"
figure 0 3 "m.vsi[C]"

page "Machine Current"
label 1 "(A)"
figure 0 4 "m.iA"
figure 0 5 "m.iB"
figure 0 6 "m.iC"

page "Supply Voltage"
label 1 "(V)"
figure 0 7 "m.Udc"
figure 0 11 "m.analog[Udc.A]"
figure 0 12 "m.analog[Udc.B]"

page "ADC Current"
label 1 "(A)"
figure 0 8  "m.analog[iA]"
figure 0 9  "m.analog[iB]"
figure 0 10 "m.analog[iC]"

page "ADC Voltage"
label 1 "(V)"
figure 0 13 "m.analog[uA]"
figure 0 14 "m.analog[uB]"
figure 0 15 "m.analog[uC]"

