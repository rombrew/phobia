#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Power consumption (Watt)'
plot    'TEL' using 1:8 title 'Plant' with lines

pause mouse close

