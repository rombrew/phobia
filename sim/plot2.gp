#!/usr/bin/gnuplot -p

set terminal x11
set grid

set xlabel 'Time (Sec)'
set ylabel 'Electrical Speed (Radian/Sec)'
plot    'TEL' using 1:5 title 'Plant' with lines

pause mouse close

