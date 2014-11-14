/*
   Phobia DC Motor Controller for RC and robotics.
   Copyright (C) 2014 Roman Belov <romblv@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _H_UKF_
#define _H_UKF_

/* Unscented Kalman Filter.
 */

#define LOW(A, I, J)		A[(I) * ((I) + 1) / 2 + (J)]

typedef struct {

	/* Size.
	 */
	int		N;
	int		M;

	/* Filter State.
	 */
	double		*P;
	double		*X;

	/* Noise.
	 */
	double		*Q;
	double		*R;

	/* Equality.
	 */
	void		(* pF) (double *Y, const double *X, const double *U);
	void		(* pH) (double *Z, const double *X);
	
	/* Kalman Gain.
	 */
	double		*K;
	
	/* Temporal.
	 */
	double		*YL, *ZL;
	double		*Z, *PZZ, *PYZ;
	
	/* End of memory.
	 */
	void		*pEnd;
}
ukfT;

ukfT *ukfAlloc(void *pMem, int N, int M);
void ukfForecast(ukfT *Kf, const double *U);
void ukfUpdate(ukfT *Kf);
void ukfCorrect(ukfT *Kf, const double *Z);

#endif /* _H_UKF_ */
