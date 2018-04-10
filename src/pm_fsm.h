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

#ifndef _H_PM_FSM_
#define _H_PM_FSM_

enum {
	PM_STATE_IDLE				= 0,
	PM_STATE_ZERO_DRIFT,
	PM_STATE_POWER_STAGE_TEST,
	PM_STATE_ADJUST_VOLTAGE,
	PM_STATE_ADJUST_CURRENT,
	PM_STATE_PROBE_CONST_R,
	PM_STATE_PROBE_CONST_L,
	PM_STATE_LU_INITIATE,
	PM_STATE_LU_SHUTDOWN,
	PM_STATE_PROBE_CONST_E,
	PM_STATE_PROBE_CONST_J,
	PM_STATE_ADJUST_HALL,
	PM_STATE_ADJUST_SENSOR,
	PM_STATE_HALT,
};

enum {
	PM_OK					= 0,
	PM_ERROR_ZERO_DRIFT_FAULT,
	PM_ERROR_NO_MOTOR_CONNECTED,
	PM_ERORR_POWER_STAGE_FAULT,
	PM_ERROR_CURRENT_LOOP_FAULT,
	PM_ERROR_OVER_CURRENT,
	PM_ERROR_ADJUST_TOLERANCE_FAULT,
	PM_ERROR_SUPPLY_VOLTAGE_LOW,
	PM_ERROR_SUPPLY_VOLTAGE_HIGH,
	PM_ERROR_LU_RESIDUAL_UNSTABLE,
	PM_ERROR_LU_SPEED_HIGH,
	PM_ERROR_LU_DRIFT_HIGH,
};

void pm_FSM(pmc_t *pm);
void pm_fsm_req(pmc_t *pm, int req);

const char *pm_strerror(int error);

#endif /* _H_PM_FSM_ */

