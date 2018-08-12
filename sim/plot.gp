#!/home/amaora/util/gp
# vi: ft=conf

load 0 -1 float 40 "/tmp/TEL"

group 0 0
deflabel 0 "(ms)"
defscale 0 1000 0

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

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.X[3]"
figure 0 11 "pm.lu_X[3:4]"

page "E. Position Error"
label 1 "(°)"
figure 0 12 "m.X[3] - pm.lu_X[3:4]"

page "pm.lu_drift_Q"
label 1 "(V)"
figure 0 14 "pm.lu_drift_Q"

page "PWM: m.PWM_A, m.PWM_B, m.PWM_C"
label 1 "(%)"
figure 0 6 "m.PWM_A"
figure 0 7 "m.PWM_B"
figure 0 8 "m.PWM_C"

page "PWM: pm.vsi_X / pm.vsi_Y"
label 0 "(V)"
label 1 "(V)"
figure 15 16 "pm.vsi_X / pm.vsi_Y"

page "PWM: pm.vsi_lpf_D, pm.vsi_lpf_Q"
label 1 "(V)"
figure 0 17 "pm.vsi_lpf_D"
figure 0 18 "pm.vsi_lpf_Q"

page "DQ Residual"
label 1 "(A)"
figure 0 19 "pm.lu_residual_D"
figure 0 20 "pm.lu_residual_Q"

page "Residual Variance"
label 1 "(A)"
figure 0 21 "pm.lu_residual_lpf"

page "pm.lu_power_lpf"
label 1 "(W)"
figure 0 25 "pm.lu_power_lpf"

page "pm.lu_mode"
figure 0 26 "pm.lu_mode"

page "pm.flux_drift_R"
figure 0 28 "pm.flux_drift_R"

