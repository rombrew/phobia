#!/usr/bin/env gp
# vi:ft=conf

load 0 0 float 40 "/tmp/pm-PWM"

group 0 0
deflabel 0 "(us)"

page "VSI Output"
figure 0 1 "m.xfet[A]"
figure 0 2 "m.xfet[B]"
figure 0 3 "m.xfet[C]"

page "Dead-Time Uncertainty"
figure 0 4 "m.xdtu[A]"
figure 0 5 "m.xdtu[B]"
figure 0 6 "m.xdtu[C]"

page "Machine Current"
label 1 "(A)"
figure 0 7 "m.iA"
figure 0 8 "m.iB"
figure 0 9 "m.iC"

page "DC link Voltage"
label 1 "(V)"
figure 0 10 "m.Udc"
figure 0 14 "m.state[Udc.A]"
figure 0 15 "m.state[Udc.B]"
figure 0 25 "m.analog_uS"
drawing line 1

page "Analog Current"
label 1 "(A)"
figure 0 11 "m.state[iA]"
figure 0 19 "m.hold_iA"
figure 0 12 "m.state[iB]"
figure 0 20 "m.hold_iB"
figure 0 13 "m.state[iC]"
figure 0 21 "m.hold_iC"

page "Analog Voltage"
label 1 "(V)"
figure 0 16 "m.state[uA]"
figure 0 22 "m.analog_uA"
figure 0 17 "m.state[uB]"
figure 0 23 "m.analog_uB"
figure 0 18 "m.state[uC]"
figure 0 24 "m.analog_uC"

