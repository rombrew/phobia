#!/usr/bin/env gp
# vi:ft=conf

load 0 0 float 100 "/tmp/pm-TLM"

group 0 0
deflabel 0 "(s)"

page "DQ Current"
label 1 "(A)"
figure 0 1 "m.iD"
figure 0 2 "m.iQ"
figure 0 12 "pm.lu_iD"
figure 0 13 "pm.lu_iQ"

page "Mechanical Speed"
label 1 "(rpm)"
figure 0 3 "m.wS_rpm"
figure 0 16 "pm.lu_wS_rpm"

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.theta"
figure 0 15 "pm.lu_Fg"

page "E.Position Error"
label 1 "(°)"
figure 0 14 "m.theta - pm.lu_Fg"

page "Motor Temperature"
label 1 "(C)"
figure 0 5 "m.Tc"

page "DC link Voltage"
label 1 "(V)"
figure 0 6 "m.Udc"
figure 0 19 "pm.const_fb_U"

page "Power Consumption"
label 1 "(W)"
figure 0 17 "m.drain_wP"
figure 0 18 "pm.watt_drain_wP"

page "PWM Output"
label 1 "(%)"
figure 0 7 "m.pwm_A"
figure 0 8 "m.pwm_B"
figure 0 9 "m.pwm_C"

page "VSI Voltage"
label 0 "(V)"
label 1 "(V)"
figure 10 11 "pm.vsi_X / pm.vsi_Y"
drawing dot 4

page "Absolute Current"
label 1 "(A)"
figure 0 20 "m.iA"
figure 0 21 "m.iB"
figure 0 22 "m.iC"

page "------------------------------"

include "/tmp/pm-auto.gp"

