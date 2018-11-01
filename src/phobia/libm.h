/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2018 Roman Belov <romblv@gmail.com>

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

#ifndef _H_LIB_M_
#define _H_LIB_M_

#define M_EPS_F			1.2E-7f
#define M_PI_F			3.14159265f
#define M_LOG2_F		0.69314718f

inline float m_fabsf(float x) { return __builtin_fabsf(x); }
inline float m_sqrtf(float x) { return __builtin_sqrtf(x); }

void m_rotf(float y[2], float r, const float x[2]);
float m_atan2f(float y, float x);
float m_sinf(float x);
float m_cosf(float x);
float m_log2f(float x);
float m_logf(float x);
float m_exp2f(float x);
float m_expf(float x);

#endif /* _H_LIB_M_ */

