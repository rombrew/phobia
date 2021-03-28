#!/home/amaora/util/gp
# vi: ft=conf

load 0 -1 float 100 "/tmp/pm-TLM"

group 0 0
deflabel 0 "(s)"

page "DQ Current"
label 1 "(A)"
figure 0 1 "m.X[0] (D)"
figure 0 2 "m.X[1] (Q)"
figure 0 12 "pm.lu_iD"
figure 0 13 "pm.lu_iQ"

page "Mechanical Speed"
label 1 "(rpm)"
figure 0 3 "m.X[2]"
figure 0 16 "pm.lu_wS"

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.X[3]"
figure 0 15 "pm.lu_F"

page "E. Position Error"
label 1 "(°)"
figure 0 14 "m.X[3] - pm.lu_F"

page "Motor Temperature"
label 1 "(C)"
figure 0 5 "m.X[4]"

page "Supply Voltage"
label 1 "(V)"
figure 0 6 "m.X[6]"
figure 0 19 "pm.const_fb_U"

page "Power Consumption"
label 1 "(W)"
figure 0 17 "m.iP"
figure 0 18 "pm.watt_lpf_wP"

page "m.PWM_[ABC]"
label 1 "(%)"
figure 0 7 "m.PWM_A"
figure 0 8 "m.PWM_B"
figure 0 9 "m.PWM_C"

page "pm.vsi_X / pm.vsi_Y"
label 0 "(V)"
label 1 "(V)"
figure 10 11 "pm.vsi_X / pm.vsi_Y"

page "------------------------------"

include "/tmp/pm-auto.gp"

