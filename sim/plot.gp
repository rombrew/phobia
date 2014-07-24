#!/usr/bin/gnuplot -p

set terminal x11
set grid

set macros
TEL = "\"TEL\" binary format=\"%40float\""

system "echo -n Number:"
N = int(system("read N; echo $N"))

if (N == 1) {

	set xlabel 'Time (Sec)'
	set ylabel 'Current (Ampere)'
	plot    @TEL using 1:2 title 'Plant D' with lines, \
		@TEL using 1:3 title 'Plant Q' with lines
#		@TEL using 1:10 title 'Sense D' with lines, \
#		@TEL using 1:11 title 'Sense Q' with lines
}

if (N == 2) {

	set xlabel 'Time (Sec)'
	set ylabel 'Electrical Speed (RPM)'
	plot    'TEL' using 1:($4/pi*30) title 'Plant' with lines, \
		'TEL' using 1:($11/pi*30) title 'Estimated' with lines
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
	plot    'TEL' using 1:(atan2(sin($5)*cos($12) - cos($5)*sin($12), \
		sin($5)*sin($12) + cos($5)*cos($12))/pi*180) title \
		'Error' with lines
}

if (N == 5) {

	set xlabel 'Time (Sec)'
	set ylabel 'Duty Cycle (%)'
	plot	@TEL using 1:($7*100) title 'D' with lines, \
		@TEL using 1:($8*100) title 'Q' with lines
}

if (N == 10) {

	set xlabel 'Time (Sec)'
	set ylabel 'Zero drift (Ampere)'
	plot    'TEL' using 1:17 title 'Estimated' with lines, \
		'TEL' using 1:18 title 'Estimated' with lines
}

if (N == 11) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding resistance (mOhm)'
	plot    'TEL' using 1:19 title 'Estimated' with lines
}

if (N == 12) {

	set xlabel 'Time (Sec)'
	set ylabel 'Winding inductance (uH)'
	plot    'TEL' using 1:20 title 'Estimated' with lines
}


if (N == 13) {

	set xlabel 'Time (Sec)'
	set ylabel 'Motor constant (uV/RpS)'
	plot    'TEL' using 1:21 title 'Estimated' with lines
}

pause mouse close

