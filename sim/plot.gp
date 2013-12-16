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
	set ylabel 'Power (Watt)'
	plot    'TEL' using 1:7 title 'Consumption' with lines, \
		'TEL' using 1:8 title 'Production' with lines
}

if (N == 6) {

	set xlabel 'Time (Sec)'
	set ylabel 'Efficiency (%)'
	plot    'TEL' using 1:($8/$7*100) title 'Plant' with lines, \
}

if (N == 7) {

	set xlabel 'Time (Sec)'
	set ylabel 'Duty Cycle (%)'
	plot    'TEL' using 1:($14*100) title 'D' with lines, \
		'TEL' using 1:($15*100) title 'Q' with lines
}

if (N == 8) {

	set xlabel 'Time (Sec)'
	set ylabel 'Current innovation (Ampere)'
	plot    'TEL' using 1:24 title 'D' with lines, \
		'TEL' using 1:25 title 'Q' with lines
}

if (N == 9) {

	set xlabel 'Time (Sec)'
	set ylabel 'Number of full turns'
	plot    'TEL' using 1:16 title 'Plant' with lines
}

pause mouse close

