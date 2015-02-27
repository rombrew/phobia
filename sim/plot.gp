#!/usr/bin/gnuplot -p

set terminal x11
set grid

set macros
TEL = "\"/tmp/TEL\" binary format=\"%40float\""

system "echo -n Number:"
N = int(system("read N; echo $N"))

if (N == 1) {

	set xlabel 'Time (Sec)'
	set ylabel 'Current (A)'
	plot    @TEL using 1:2 title 'Plant D' with lines, \
		@TEL using 1:3 title 'Plant Q' with lines, \
		@TEL using 1:10 title 'Estimated D' with lines, \
		@TEL using 1:11 title 'Estimated Q' with lines
}

if (N == 2) {

	set xlabel 'Time (Sec)'
	set ylabel 'Electrical Speed (RPM)'
	plot    @TEL using 1:($4*30/pi) title 'Plant' with lines, \
		@TEL using 1:($14*30/pi) title 'Estimated' with lines
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

	set xlabel 'Time (Sec)'
	set ylabel 'Load Torque'
	plot	@TEL using 1:21 title 'Load Torque' with lines 
}

if (N == 7) {

	set xlabel 'Time (Sec)'
	set ylabel 'BEMF Constant (Kv)'
	plot	@TEL using 1:(60/($19*2*sqrt(3)*pi*$20)) title 'Kv' with lines 
}

if (N == 8) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding Resistance (mOhm)'
	plot	@TEL using 1:($16*1e+3) title 'R' with lines
}

if (N == 9) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding Inductance (uH)'
	plot	@TEL using 1:($17*1e+6) title 'Ld' with lines, \
		@TEL using 1:($18*1e+6) title 'Lq' with lines
}

if (N == 10) {

	set xlabel 'Time (Sec)'
	set ylabel 'Supply Voltage'
	plot	@TEL using 1:16 title 'U' with lines 
}

if (N == 11) {

	set xlabel 'Time (Sec)'
	set ylabel 'Zero Drift (A)'
	plot	@TEL using 1:21 title 'A' with lines, \
		@TEL using 1:22 title 'B' with lines
}

pause mouse close

