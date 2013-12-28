#!/usr/bin/gnuplot -p

set terminal x11
set grid

system "echo -n Number:"
N = int(system("read N; echo $N"))

if (N == 1) {

	set xlabel 'Time (Sec)'
	set ylabel 'Current (Ampere)'
	plot    'TEL' using 1:2 title 'Plant D' with lines, \
		'TEL' using 1:3 title 'Plant Q' with lines, \
		'TEL' using 1:9 title 'Estimated D' with lines, \
		'TEL' using 1:10 title 'Estimated Q' with lines
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
	plot    'TEL' using 1:($5/pi*180) title 'Plant' with lines, \
		'TEL' using 1:($12/pi*180) title 'Estimated' with lines
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
	plot    'TEL' using 1:($14*100) title 'D' with lines, \
		'TEL' using 1:($15*100) title 'Q' with lines
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

