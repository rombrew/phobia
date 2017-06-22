#!/home/amaora/util/gp
# vi: ft=conf

font 18 "/usr/share/fonts/corefonts/cour.ttf"
screenpath "/tmp"
windowsize 1200 900

batch 1
defstyle line 1

load 0 500 text "/dev/rfcomm0"

group 0 0
deflabel 0 "Time (tick)"

page 1 "DQ Current"
label 1 "Current (mA)"
figure -1 0 "D"
figure -1 1 "Q"

page 2 "Electrical Position"
label 0 "Tick"
label 1 "Position"
figure -1 2 "Cos"
figure -1 3 "Sin"

page 3 "Mechanical Speed"
label 0 "Tick"
label 1 "Mechanical Speed (RPM)"
figure -1 4 "S"

page 4 "Q Drift"
label 0 "Tick"
label 1 "Q Drift (mV)"
figure -1 5 "Q"

page 5 "DQ Residual"
label 0 "Tick"
label 1 "Residual (mV)"
figure -1 6 "D"
figure -1 7 "Q"


