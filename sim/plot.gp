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

page "pm.flux_drift_Q"
label 1 "(V)"
figure 0 14 "pm.flux_drift_Q"
map 0 2

page "PWM: m.PWM_[ABC]"
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

page "DQ Residue"
label 1 "(A)"
figure 0 19 "pm.flux_residue_D"
figure 0 20 "pm.flux_residue_Q"

page "Residue Variance"
label 1 "(A)"
figure 0 21 "pm.flux_residue_lpf"

page "Power Consumption"
label 1 "(W)"
figure 0 22 "m.iP"
figure 0 23 "pm.vsi_lpf_watt"

page "pm.vsi_clamp_to_GND"
figure 0 24 "pm.vsi_clamp_to_GND"

page "pm.vsi_zone_[ABC]"
figure 0 25 "pm.vsi_zone_A"
figure 0 26 "pm.vsi_zone_B"
figure 0 27 "pm.vsi_zone_C"

page "pm.tvse_residue_[XY]"
label 1 "(V)"
figure 0 28 "pm.tvse_residue_X"
figure 0 29 "pm.tvse_residue_Y"

page "pm.lu_mode"
figure 0 30 "pm.lu_mode"
