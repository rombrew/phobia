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

page "Residual"
label 1 "(A)"
figure 0 19 "pm.lu_residual_D"
figure 0 20 "pm.lu_residual_Q"

page "pm.n_power_watt"
label 1 "(W)"
figure 0 24 "pm.n_power_watt"

page "pm.drift_Q"
label 1 "(V)"
figure 0 23 "pm.drift_Q"

page "pm.n_temperature_c"
label 1 "(C)"
figure 0 25 "pm.n_temperature_c"

page "pm.pb_temp"
label 1 "(%)"
figure 0 28 "pm.pb_temp"
figure 0 29 "pm.pb_temp"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x1)"
label 1 "(%)"
figure 0 40 "pm.bemf_DFT[2]"
figure 0 41 "pm.bemf_DFT[3]"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x2)"
label 1 "(%)"
figure 0 42 "pm.bemf_DFT[6]"
figure 0 43 "pm.bemf_DFT[7]"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x3)"
label 1 "(%)"
figure 0 44 "pm.bemf_DFT[10]"
figure 0 45 "pm.bemf_DFT[11]"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x4)"
label 1 "(%)"
figure 0 46 "pm.bemf_DFT[14]"
figure 0 47 "pm.bemf_DFT[15]"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x5)"
label 1 "(%)"
figure 0 48 "pm.bemf_DFT[18]"
figure 0 49 "pm.bemf_DFT[19]"

page "pm.bemf_DFT[2], pm.bemf_DFT[2] (x6)"
label 1 "(%)"
figure 0 50 "pm.bemf_DFT[22]"
figure 0 51 "pm.bemf_DFT[23]"
