/*
   Phobia Motor Controller for RC and robotics.
   Copyright (C) 2016 Roman Belov <romblv@gmail.com>

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

#ifndef _H_AP_
#define _H_AP_

#include "sh.h"

SH_DEF(ap_identify_base);
SH_DEF(ap_identify_const_R_abc);
SH_DEF(ap_identify_const_E);
SH_DEF(ap_J_measure_T);
SH_DEF(ap_identify_const_J);
SH_DEF(ap_blind_spinup);
SH_DEF(ap_probe_base);

#endif /* _H_AP_ */

