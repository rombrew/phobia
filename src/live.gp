#!/home/amaora/util/gp
# vi: filetype=conf

font 14 "/usr/share/fonts/corefonts/cour.ttf"
batch 1
scheme 0

load 0 500 text "/dev/rfcomm0"

page 1 "DQ Current"
label 0 "Tick"
label 1 "Current (mA)"
figure 0 -1 0 0 1 "D"
figure 0 -1 1 0 1 "Q"

page 2 "Mechanical Speed"
label 0 "Tick"
label 1 "Mechanical Speed (RPM)"
figure 0 -1 4 0 1 "S"

page 3 "Q Drift"
label 0 "Tick"
label 1 "Q Drift (mV)"
figure 0 -1 5 0 1 "Q"

page 4 "DQ Residual"
label 0 "Tick"
label 1 "Residual (mV)"
figure 0 -1 6 0 1 "D"
figure 0 -1 7 0 1 "Q"


