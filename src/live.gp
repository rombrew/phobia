#!/home/amaora/util/gp
# vi: ft=conf

font 18 "/usr/share/fonts/corefonts/cour.ttf"
screenpath "/tmp"
windowsize 1200 900

batch 1
defstyle line 1

#load 0 1024 text "/dev/rfcomm0"
load 0 1024 text "/tmp/tel.txt"

group 0 0
deflabel 0 "Time (tick)"

page "DQ Current"
label 1 "(mA)"
figure -1 0 "pm.lu_X[0]"
figure -1 1 "pm.lu_X[1]"

page "E. Position"
figure -1 2 "pm.lu_X[2]"
figure -1 3 "pm.lu_X[3]"

page "Mechanical Speed"
label 1 "(rpm)"
figure -1 4 "pm.lu_X[4]"

page "Q Drift"
label 1 "(mV)"
figure -1 5 "pm.drift_Q"

page "DQ Residual"
label 1 "(mA)"
figure -1 6 "pm.lu_residual_D"
figure -1 7 "pm.lu_residual_Q"

page "AB Current"
label 1 "(mA)"
figure -1 8 "pm.fb_iA"
figure -1 9 "pm.fb_iB"

page "Supply Voltage"
label 1 "(mV)"
figure -1 10 "pm.const_U"

page "Power Consumption"
label 1 "(W/10)"
figure -1 11 "pm.n_power_watt"



