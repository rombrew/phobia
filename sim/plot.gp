#!/home/amaora/util/gp
# vi: ft=conf

font 16 "/usr/share/fonts/corefonts/cour.ttf"
batch 20000
scheme 0

load 0 -1 float 40 "/tmp/TEL"

page 1 "DQ Current"
label 0 "Time (Sec)"
label 1 "Current (Ampere)"
figure 0 1 "Plant D"
figure 0 2 "Plant Q"
figure 0 9 "Estimated D"
figure 0 10 "Estimated Q"

page 2 "Mechanical Speed"
label 0 "Time (Sec)"
label 1 "Mechanical Speed (RPM)"
figure 0 3 "Plant"
figure 0 13 "Estimated"
figure 0 24 "Nonl X4"

page 3 "Electrical Position"
label 0 "Time (Sec)"
label 1 "Electrical Position (Degree)"
figure 0 4 "Plant"
figure 0 11 "Estimated"

page 4 "Electrical Position Error"
label 0 "Time (Sec)"
label 1 "Error (Degree)"
figure 0 12 "Error"

page 5 "PWM Duty Cycle"
label 0 "Time (Sec)"
label 1 "Duty Cycle (%)"
figure 0 6 "A"
figure 0 7 "B"
figure 0 8 "C"

page 6 "VSI Voltage"
label 0 "X (Volt)"
label 1 "Y (Volt)"
figure 15 16 "VSI"

page 7 "DQ Residual"
label 0 "Time (Sec)"
label 1 "Residual (Ampere)"
figure 0 17 "D"
figure 0 18 "Q"

page 8 "Power"
label 0 "Time (Sec)"
label 1 "Power (Watt)"
figure 0 22 "W"

page 9 "Drift Q"
label 0 "Time (Sec)"
label 1 "Drift (Volt)"
figure 0 21 "Q"

