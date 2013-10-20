#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Duty cycle (%)'
plot    'TEL' using 1:9 title 'Plant A' with lines, \
	'TEL' using 1:10 title 'Plant B' with lines, \
	'TEL' using 1:11 title 'Plant C' with lines

pause mouse close

