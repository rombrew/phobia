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
figure 0 10 "pm.lu_X[0] (D)"
figure 0 11 "pm.lu_X[1] (Q)"

page "Mechanical Speed"
label 1 "(rpm)"
figure 0 3 "m.X[2]"
figure 0 14 "pm.lu_X[4]"
figure 0 34 "pm.s_track"

page "Electrical Position"
label 1 "(°)"
figure 0 4 "m.X[3]"
figure 0 12 "pm.lu_X[3:4]"

page "E. Position Error"
label 1 "(°)"
figure 0 13 "m.X[3] - pm.lu_X[3:4]"

page "Supply Voltage"
label 1 "(V)"
figure 0 6 "m.X[6]"
figure 0 32 "pm.const_lpf_U"

page "FLUX: pm.flux_drift_Q"
label 1 "(V)"
figure 0 15 "pm.flux_drift_Q"

page "LU: pm.lu_mode"
figure 0 33 "pm.lu_mode"

page "PWM: m.PWM_[ABC]"
label 1 "(%)"
figure 0 7 "m.PWM_A"
figure 0 8 "m.PWM_B"
figure 0 9 "m.PWM_C"

page "PWM: pm.vsi_X / pm.vsi_Y"
label 0 "(V)"
label 1 "(V)"
figure 16 17 "pm.vsi_X / pm.vsi_Y"

page "PWM: pm.vsi_lpf_D, pm.vsi_lpf_Q"
label 1 "(V)"
figure 0 18 "pm.vsi_lpf_D"
figure 0 19 "pm.vsi_lpf_Q"

page "PWM: pm.vsi_current_ZONE"
figure 0 20 "pm.vsi_current_ZONE"

page "PWM: pm.vsi_voltage_ZONE"
figure 0 21 "pm.vsi_voltage_ZONE"

page "VM: pm.vm_[ABC]"
figure 0 22 "pm.vm_A"
figure 0 23 "pm.vm_B"
figure 0 24 "pm.vm_C"

page "VM: pm.vm_residue_[XY]"
label 1 "(V)"
figure 0 25 "pm.vm_residue_X"
figure 0 26 "pm.vm_residue_Y"

page "FLUX: DQ Residue"
label 1 "(A)"
figure 0 27 "pm.flux_residue_D"
figure 0 28 "pm.flux_residue_Q"

page "FLUX: Residue Variance"
label 1 "(A²)"
figure 0 29 "pm.flux_residue_lpf"

page "Power Consumption"
label 1 "(W)"
figure 0 30 "m.iP"
figure 0 31 "pm.vsi_lpf_watt"

