#!/usr/bin/gnuplot -p

set terminal x11
set grid

set macros
TEL = "\"/tmp/TEL\" binary format=\"%40float\""

system "echo -n Number:"
N = int(system("read N; echo $N"))

if (N == 1) {

	set xlabel 'Time (Sec)'
	set ylabel 'Current (Ampere)'
	plot    @TEL using 1:2 title 'Plant D' with lines, \
		@TEL using 1:3 title 'Plant Q' with lines, \
		@TEL using 1:10 title 'Estimated D' with lines, \
		@TEL using 1:11 title 'Estimated Q' with lines
}

if (N == 2) {

	set xlabel 'Time (Sec)'
	set ylabel 'Mechanical Speed (RPM)'
	plot    @TEL using 1:($4/$27*30/pi) title 'Plant' with lines, \
		@TEL using 1:($14/$27*30/pi) title 'Estimated' with lines, \
		@TEL using 1:($32/$27*30/pi) title 'Set Point' with lines
}

if (N == 3) {

	set xlabel 'Time (Sec)'
	set ylabel 'Electrical Position (Degree)'
	plot	@TEL using 1:($5*180/pi) title 'Plant' with lines, \
		@TEL using 1:($12*180/pi) title 'Estimated' with lines
}

if (N == 4) {

	set xlabel 'Time (Sec)'
	set ylabel 'Electrical Position (Degree)'
	plot    @TEL using 1:($13*180/pi) title 'Error' with lines
}

if (N == 5) {

	set xlabel 'Time (Sec)'
	set ylabel 'Duty Cycle (%)'
	plot	@TEL using 1:($7*100) title 'A' with lines, \
		@TEL using 1:($8*100) title 'B' with lines, \
		@TEL using 1:($9*100) title 'C' with lines
}

if (N == 6) {

	set xlabel 'X'
	set ylabel 'Y'
	plot	@TEL using 15:16 title 'VSI voltage (Volt)' with points
}

if (N == 7) {

	set xlabel 'Time (Sec)'
	set ylabel 'Residual (A)'
	plot	@TEL using 1:17 title 'D' with lines, \
		@TEL using 1:18 title 'Q' with lines
}

if (N == 8) {

	set xlabel 'Time (Sec)'
	set ylabel 'Load Torque'
	plot	@TEL using 1:28 title 'var_M' with lines
}

if (N == 9) {

	set xlabel 'Time (Sec)'
	set ylabel 'Q Drift (Volt)'
	plot	@TEL using 1:21 title 'drift_Q' with lines
}

if (N == 10) {

	set xlabel 'Time (Sec)'
	set ylabel 'Supply (Volt)'
	plot	@TEL using 1:22 title 'const_U' with lines
}

if (N == 11) {

	set xlabel 'Time (Sec)'
	set ylabel 'BEMF Constant (RPM/V)'
	plot	@TEL using 1:(60/($23*2*sqrt(3)*pi*$27)) title 'const_Kv' with lines
}

if (N == 12) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding Resistance (mOhm)'
	plot	@TEL using 1:($24*1e+3) title 'const_R' with lines
}

if (N == 13) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding Inductance (uH)'
	plot	@TEL using 1:($25*1e+6) title 'const_Ld' with lines, \
		@TEL using 1:($26*1e+6) title 'const_Lq' with lines
}

if (N == 14) {

	set xlabel 'Time (Sec)'
	set ylabel 'Zero Drift (A)'
	plot	@TEL using 1:19 title 'A' with lines, \
		@TEL using 1:20 title 'B' with lines
}

if (N == 15) {

	set xlabel 'Time (Sec)'
	set ylabel 'Residial variance (A)'
	plot	@TEL using 1:33 title 'DQ' with lines,
}

pause mouse close

