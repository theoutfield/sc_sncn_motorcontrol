
/**
 * \file velocity_ctrl_client.xc
 * \brief Velocity Control Loop Client functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "velocity_ctrl_client.h"
#include "refclk.h"
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"

//#define debug_print

int init_velocity_control(chanend c_velocity_ctrl)
{
	int ctrl_state = INIT_BUSY;

	while(1)
	{
		ctrl_state = check_velocity_ctrl_state(c_velocity_ctrl);
		if(ctrl_state == INIT_BUSY)
		{
			enable_velocity_ctrl(c_velocity_ctrl);
		}
		if(ctrl_state == INIT)
		{
			#ifdef debug_print
				printstrln("velocity control intialized");
			#endif
			break;
		}
	}
	return ctrl_state;
}

//internal
void set_velocity(int target_velocity, chanend c_velocity_ctrl) {
	VELOCITY_CTRL_WRITE(SET_VELOCITY_TOKEN);
	VELOCITY_CTRL_WRITE(target_velocity);
}

int get_velocity(chanend c_velocity_ctrl) {
	int velocity;
	VELOCITY_CTRL_WRITE(GET_VELOCITY_TOKEN);
	VELOCITY_CTRL_READ(velocity);
	return velocity;
}

int max_speed_limit(int velocity, int max_speed) {
	if (velocity > max_speed) {
		velocity = max_speed;
		return velocity;
	} else if (velocity < -max_speed) {
		velocity = -max_speed;
		return velocity;
	} else if (velocity >= -max_speed && velocity <= max_speed) {
		return velocity;
	}
}

//csv mode function
void set_velocity_csv(csv_par &csv_params, int target_velocity,
		int velocity_offset, int torque_offset, chanend c_velocity_ctrl)
{
	set_velocity( max_speed_limit(	(target_velocity + velocity_offset) * csv_params.polarity, csv_params.max_motor_speed  ), c_velocity_ctrl );
}

void set_velocity_ctrl_param(ctrl_par &velocity_ctrl_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_CTRL_PARAMETER);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kp_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kp_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Ki_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Ki_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kd_n);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Kd_d);
	VELOCITY_CTRL_WRITE(velocity_ctrl_params.Integral_limit);
}

void set_velocity_ctrl_hall_param(hall_par &hall_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_CTRL_HALL);
	VELOCITY_CTRL_WRITE(hall_params.pole_pairs);
	VELOCITY_CTRL_WRITE(hall_params.max_ticks);
	VELOCITY_CTRL_WRITE(hall_params.max_ticks_per_turn);
}

void set_velocity_ctrl_qei_param(qei_par &qei_params, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_CTRL_QEI);
	VELOCITY_CTRL_WRITE(qei_params.max_ticks);
	VELOCITY_CTRL_WRITE(qei_params.index);
	VELOCITY_CTRL_WRITE(qei_params.real_counts);
	VELOCITY_CTRL_WRITE(qei_params.max_ticks_per_turn);
	VELOCITY_CTRL_WRITE(qei_params.poles);
}

void set_velocity_sensor(int sensor_used, chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SENSOR_SELECT);
	VELOCITY_CTRL_WRITE(sensor_used);
}

void set_velocity_filter(chanend c_velocity_ctrl, filter_par &filter_params)
{
	VELOCITY_CTRL_WRITE(SET_VELOCITY_FILTER);
	VELOCITY_CTRL_WRITE(filter_params.filter_length);
}

void enable_velocity_ctrl(chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(ENABLE_VELOCITY_CTRL);
	VELOCITY_CTRL_WRITE(1);
}

void shutdown_velocity_ctrl(chanend c_velocity_ctrl)
{
	VELOCITY_CTRL_WRITE(SHUTDOWN_VELOCITY_CTRL);
	VELOCITY_CTRL_WRITE(0);
}

int check_velocity_ctrl_state(chanend c_velocity_ctrl)
{
	int state;
	VELOCITY_CTRL_WRITE(VELOCITY_CTRL_STATUS);
	VELOCITY_CTRL_READ(state);
	return state;
}
