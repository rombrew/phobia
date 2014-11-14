#!/usr/bin/octave -q

T = 1 / 20e+3;

U = 12;
R = 74e-3;
L = 44e-6;
E = 66e-5;
Zp = 11;
J = 10e-5;

w = 100 * 2 * pi;

iD = 0;
iQ = 10;

A = [	1-R*T/L		w*T		0	iQ*T		0; ...
	-w*T		1-R*T/L		0	-E*T/L-iD*T	0; ...
	0		0		1	T		0; ...
	0		E*Zp*Zp*1.5*T/J	0	1		-Zp*T/J; ...
	0		0		0	0		1;]

C = [	1		0		iQ	0		0; ...
	0		1		-iD	0		0;]

rk = [C; C*A; C*A^2; C*A^3; C*A^4]
rk = rank(rk)

P = diag([1e-4 1e-4 1e-4 1e-4 1e-4]);
Q = diag([1e-8 1e-8 1e-8 1e-8 1e-2]);
R = diag([1e-2 1e-2]);

for i=1:10000

	P = A * P * A' + Q;
	S = C * P * C' + R;
	K = P * C' / S;
	P = P - K * C * P;
end

P
K

abs(eig(A - K*C))

