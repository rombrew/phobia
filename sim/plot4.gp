#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Temperature (Celsius)'
plot    'TEL' using 1:7 title 'Plant A' with lines

pause mouse close

