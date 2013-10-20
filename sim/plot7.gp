#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Drift (Ampere)'
plot    'TEL' using 1:14 title 'Sensor A' with lines, \
	'TEL' using 1:15 title 'Sensor C' with lines

pause mouse close

