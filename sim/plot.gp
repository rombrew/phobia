#!/home/amaora/util/gp
# vi: ft=conf

load 0 -1 float 80 "/tmp/TEL"

group 0 0
deflabel 0 "(s)"
defscale 0 1.0 0

page "DQ Current"
label 1 "(A)"
figure 0 1 "m.X[0] (D)"
figure 0 2 "m.X[1] (Q)"
figure 0 12 "pm.lu_iD"
figure 0 13 "pm.lu_iQ"

page "Mechanical Speed"
label 1 "(rpm)"
figure 0 3 "m.X[2]"
figure 0 21 "pm.lu_wS"
figure 0 51 "pm.s_setpoint"

page "pm.flux_wS"
label 1 "(rpm)"
figure 0 22 "pm.flux_wS"

page "pm.forced_wS"
label 1 "(rpm)"
figure 0 23 "pm.forced_wS"

page "pm.hfi_wS"
label 1 "(rpm)"
figure 0 24 "pm.hfi_wS"

page "pm.hall_wS"
label 1 "(rpm)"
figure 0 25 "pm.hall_wS"

page "pm.qenc_wS"
label 1 "(rpm)"
figure 0 26 "pm.qenc_wS"

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.X[3]"
figure 0 15 "pm.lu_F"

page "E. Position Error"
label 1 "(°)"
figure 0 14 "m.X[3] - pm.lu_F"

page "pm.flux_F"
label 1 "(°)"
figure 0 16 "pm.flux_F"

page "pm.forced_F"
label 1 "(°)"
figure 0 17 "pm.forced_F"

page "pm.hfi_F"
label 1 "(°)"
figure 0 18 "pm.hfi_F"

page "pm.hall_F"
label 1 "(°)"
figure 0 19 "pm.hall_F"

page "pm.qenc_F"
label 1 "(°)"
figure 0 20 "pm.qenc_F"

page "Motor Temperature"
label 1 "(C)"
figure 0 5 "m.X[4]"

page "Supply Voltage"
label 1 "(V)"
figure 0 6 "m.X[6]"
figure 0 47 "pm.const_fb_U"

page "Power Consumption"
label 1 "(W)"
figure 0 45 "m.iP"
figure 0 46 "pm.watt_lpf_wP"

page "m.PWM_[ABC]"
label 1 "(%)"
figure 0 7 "m.PWM_A"
figure 0 8 "m.PWM_B"
figure 0 9 "m.PWM_C"

page "pm.vsi_X / pm.vsi_Y"
label 0 "(V)"
label 1 "(V)"
figure 28 29 "pm.vsi_X / pm.vsi_Y"

page "pm.vsi_EU"
figure 0 27 "pm.vsi_EU"

page "pm.vsi_IF"
figure 0 30 "pm.vsi_IF"

page "pm.vsi_UF"
figure 0 31 "pm.vsi_UF"

page "m.pulse_HS"
figure 0 10 "m.pulse_HS"

page "m.pulse_EP"
figure 0 11 "m.pulse_EP"

page "pm.tvm_[ABC]"
figure 0 32 "pm.tvm_A"
figure 0 33 "pm.tvm_B"
figure 0 34 "pm.tvm_C"

page "pm.tvm_[XY]"
label 1 "(V)"
figure 0 35 "pm.tvm_DX"
figure 0 36 "pm.tvm_DY"

page "pm.lu_mode"
figure 0 37 "pm.lu_mode"

page "pm.lu_TIM"
figure 0 38 "pm.lu_TIM"

page "pm.qenc_TIM"
figure 0 54 "pm.qenc_TIM"

page "pm.flux_E"
label 1 "(Wb)"
figure 0 39 "pm.flux_E"

page "pm.flux_H"
figure 0 40 "pm.flux_H"

page "pm.flux[H].lpf_E"
figure 0 41 "pm.flux[H].lpf_E"

page "pm.hfi_polarity"
figure 0 42 "pm.hfi_polarity"

page "pm.watt_lpf_D, pm.watt_lpf_Q"
label 1 "(V)"
figure 0 43 "pm.watt_lpf_D"
figure 0 44 "pm.watt_lpf_Q"

page "pm.i_setpoint_D"
label 1 "(A)"
figure 0 48 "pm.i_setpoint_D"

page "pm.i_setpoint_Q"
label 1 "(A)"
figure 0 49 "pm.i_setpoint_Q"

page "pm.weak_D"
label 1 "(A)"
figure 0 50 "pm.weak_D"

page "pm.s_setpoint"
label 1 "(rpm)"
figure 0 51 "pm.s_setpoint"

page "pm.s_track"
label 1 "(rpm)"
figure 0 52 "pm.s_track"

page "pm.s_integral"
label 1 "(A)"
figure 0 53 "pm.s_integral"

