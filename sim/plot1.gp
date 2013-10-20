#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Current (Ampere)'
plot    'TEL' using 1:2 title 'Plant A' with lines, \
	'TEL' using 1:3 title 'Plant B' with lines, \
	'TEL' using 1:4 title 'Plant B' with lines

pause mouse close

