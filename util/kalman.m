#!/usr/bin/octave -q

pkg load symbolic

syms Ix Iy th w Rs Ld Lq lam T Ux Uy bias real

% We derive Jacobian matrix \F from PMSM equations in stationary frame.
%
Ath = [cos(th) sin(th); -sin(th) cos(th)];		% XY -> DQ
Awt = [-sin(2*th) cos(2*th); cos(2*th) sin(2*th)];

L = simplify(Ath' * diag([1/Ld; 1/Lq]) * Ath);
U = simplify([Ux; Uy] - Rs * [Ix; Iy] ...
		- w * (Ld - Lq) * Awt * [Ix; Iy] ...
		+ Ath' * [0; - w * lam + bias]);

f = [Ix; Iy; th; w; bias] + [L * U; w; 0; 0] * T;
h = [Ix; Iy];

F = simplify([diff(f, Ix) diff(f, Iy) diff(f, th) diff(f, w) diff(f, bias)]);
H = simplify([diff(h, Ix) diff(h, Iy) diff(h, th) diff(h, w) diff(h, bias)]);

F = simplify(eval(F));
H = eval(H);

% Try to construct the same matrix \F from scratch.
%
Fb = sym(eye(5,5));
Fw = sym(zeros(5,5));

u(1) = (1/Ld - 1/Lq) * (Ux - Rs * Ix);
u(2) = (1/Ld - 1/Lq) * (Uy - Rs * Iy);
u(3) = (w*lam - bias) / Lq;

u(4) = Ix * cos(2*th) + Iy * sin(2*th);
u(5) = Iy * cos(2*th) - Ix * sin(2*th);

Fb(3,4) = T;

Fb(1,1) = 1 - T * Rs * (1/Ld - sin(th)^2 * (1/Ld - 1/Lq));
Fb(1,2) =   - T * Rs * sin(th) * cos(th) * (1/Ld - 1/Lq);
Fb(2,1) =   - T * Rs * sin(th) * cos(th) * (1/Ld - 1/Lq);
Fb(2,2) = 1 - T * Rs * (1/Lq + sin(th)^2 * (1/Ld - 1/Lq));

Fb(1,3) = T * (u(2) * cos(th*2) - u(1) * sin(th*2) + u(3) * cos(th));
Fb(2,3) = T * (u(1) * cos(th*2) + u(2) * sin(th*2) + u(3) * sin(th));

Fb(1,4) =   T * lam * sin(th) / Lq;
Fb(2,4) = - T * lam * cos(th) / Lq;

Fb(1,5) = - T * sin(th) / Lq;
Fb(2,5) =   T * cos(th) / Lq;

Fw(1,1) =   T * w * sin(th) * cos(th)        * (Ld/Lq - Lq/Ld);
Fw(1,2) =   T * w * ((Ld/Lq - 1) - cos(th)^2 * (Ld/Lq - Lq/Ld));
Fw(2,1) =   T * w * ((1 - Lq/Ld) - cos(th)^2 * (Ld/Lq - Lq/Ld));
Fw(2,2) = - T * w * sin(th) * cos(th)        * (Ld/Lq - Lq/Ld);

Fw(1,3) =   T * w * u(4) * (Ld/Lq - Lq/Ld);
Fw(2,3) = - T * w * u(5) * (Ld/Lq - Lq/Ld);

Fw(1,4) = - T / 2 * (Iy * (Ld - Lq) - u(5) * (Ld + Lq)) * (1/Ld - 1/Lq);
Fw(2,4) =   T / 2 * (Ix * (Ld - Lq) + u(4) * (Ld + Lq)) * (1/Ld - 1/Lq);

% Check for equality.
%
simplify(F - (Fb + Fw))

