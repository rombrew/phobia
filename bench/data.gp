#!/usr/bin/env gp
# vi:ft=conf

load 0 0 float 10 "/tmp/pm-mldata"

group 0 0
deflabel 0 "(s)"

page "XY Current"
label 1 "(A)"
figure -1 0 "iX"
figure -1 1 "iY"

page "XY Voltage"
label 1 "(V)"
figure -1 2 "uX"
figure -1 3 "uY"

page "SIN/COS Position"
figure -1 4 "COS"
figure -1 5 "SIN"

page "Electrical Speed"
figure -1 6 "\omega"

page "Machine Temperature"
label 1 "(C)"
figure -1 7 "Tc"
