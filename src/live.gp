#!/home/amaora/util/gp
# vi: ft=conf

font 18 "/usr/share/fonts/corefonts/cour.ttf"
screenpath "/tmp"
windowsize 1200 900

batch 1
defstyle line 1

#load 0 1024 text "/dev/rfcomm0"
load 0 1024 text "/tmp/tel.txt"

group 0 -1
deflabel 0 "Time (s)"
defscale 0 1E-3 0.

group 1 0 1 6 7 10 11
deflabel 1 "(A)"
defscale 1 1E-2 0.

group 2 5 8 9 12
deflabel 2 "(V)"
defscale 2 2E-3 0.

group 3 4 14
deflabel 3 "(rpm)"

page "DQ Current"
figure -1 0 "pm.lu_X[0]"
figure -1 1 "pm.lu_X[1]"

page "E. Position"
figure -1 2 "pm.lu_X[2]"
figure -1 3 "pm.lu_X[3]"

page "Mechanical Speed"
figure -1 4 "pm.lu_X[4]"
figure -1 14 "pm.s_nonl_X4"

page "Q Drift"
figure -1 5 "pm.drift_Q"
scale 1 1E-3 0.

page "DQ Residual"
figure -1 6 "pm.lu_residual_D"
scale 1 1E-3 0.
figure -1 7 "pm.lu_residual_Q"
scale 1 1E-3 0.

page "DQ Voltage"
figure -1 8 "pm.vsi_D"
figure -1 9 "pm.vsi_Q"

page "AB Current"
figure -1 10 "pm.fb_iA"
figure -1 11 "pm.fb_iB"

page "Supply Voltage"
figure -1 12 "pm.const_U"

page "Power Consumption"
label 1 "(W)"
figure -1 13 "pm.n_power_watt"
scale 1 1E-1 0.

