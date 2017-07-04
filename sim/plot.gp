#!/home/amaora/util/gp
# vi: ft=conf

font 18 "/usr/share/fonts/corefonts/cour.ttf"
screenpath "/tmp"
windowsize 1200 900

batch 20000
defstyle line 1

load 0 -1 float 80 "/tmp/TEL"

group 0 0
deflabel 0 "Time (s)"

page "DQ Current"
label 1 "(A)"
figure 0 1 "m.X[0] (D)"
figure 0 2 "m.X[1] (Q)"
figure 0 9 "pm.lu_X[0] (D)"
figure 0 10 "pm.lu_X[1] (Q)"

page "Mechanical Speed"
label 1 "(rpm)"
figure 0 3 "m.X[2]"
figure 0 13 "pm.lu_X[4]"
figure 0 14 "pm.s_nonl_X4"

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.X[3]"
figure 0 11 "pm.lu_X[3:4]"

page "E. Position Error"
label 1 "(°)"
figure 0 12 "m.X[3] - pm.lu_X[3:4]"

page "PWM: m.uA, m.uB, m.uC"
label 1 "(%)"
figure 0 6 "m.uA"
figure 0 7 "m.uB"
figure 0 8 "m.uC"

page "PWM: pm.vsi_X / pm.vsi_Y"
label 0 "(V)"
label 1 "(V)"
figure 15 16 "pm.vsi_X / pm.vsi_Y"

page "PWM: pm.vsi_D, pm.vsi_Q"
label 1 "(V)"
figure 0 17 "pm.vsi_D"
figure 0 18 "pm.vsi_Q"

page "DQ Residual"
label 1 "(A)"
figure 0 19 "pm.lu_residual_D"
figure 0 20 "pm.lu_residual_Q"

page "Residual Variance"
label 1 "(A)"
figure 0 21 "pm.lu_residual_variance"

page "pm.drift_Q"
label 1 "(V)"
figure 0 24 "pm.drift_Q"

page "pm.n_power_watt"
label 1 "(W)"
figure 0 25 "pm.n_power_watt"

page "pm.n_temperature_c"
label 1 "(C)"
figure 0 26 "pm.n_temperature_c"

