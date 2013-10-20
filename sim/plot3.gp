#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Electrical position (Radian)'
plot    'TEL' using 1:6 title 'Plant' with lines

pause mouse close

