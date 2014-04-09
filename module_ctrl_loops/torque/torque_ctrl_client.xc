
/**
 * \file  torque_ctrl_client.xc
 * \brief Torque Control Loop Client functions
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

#include "torque_ctrl_client.h"
#include <refclk.h>
#include <print.h>
#include <drive_config.h>
#include <internal_config.h>
#include <bldc_motor_config.h>
//#define debug_print

int init_torque_control(chanend c_torque_ctrl)
{
	int ctrl_state = INIT_BUSY;

	while(1)
	{
		ctrl_state = check_torque_ctrl_state(c_torque_ctrl);
		if(ctrl_state == INIT_BUSY)
		{
			enable_torque_ctrl(c_torque_ctrl);
		}
		if(ctrl_state == INIT)
		{
			#ifdef debug_print
				printstrln("torque control intialized");
			#endif
			break;
		}
	}
	return ctrl_state;
}

int get_torque(chanend c_torque_ctrl)
{
	int torque;
	TORQUE_CTRL_WRITE(GET_TORQUE_TOKEN);
	TORQUE_CTRL_READ(torque);
	return (torque);
}

void set_torque(int torque, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_TOKEN);
	TORQUE_CTRL_WRITE(torque);
	return;
}

void send_torque_init_state(chanend c_torque_ctrl, int init_state)
{
	int command;
	select
	{
		case c_torque_ctrl:> command:
			if(command == CHECK_BUSY)
			{
				c_torque_ctrl <: init_state;
			}
			break;
	}
}

int torque_limit(int torque, int max_torque_limit)
{
	if(torque > max_torque_limit) //adc range // (5 * DC900_RESOLUTION)/2
	{
		return max_torque_limit;
	}
	else if(torque < 0 - max_torque_limit)
	{
		return (0 - max_torque_limit);
	}
	else if(torque >= -max_torque_limit && torque <= max_torque_limit)
	{
		return torque;
	}
}

void set_torque_cst(cst_par &cst_params, int target_torque, int torque_offset, chanend c_torque_ctrl)
{
	set_torque( torque_limit( (target_torque + torque_offset) * cst_params.polarity ,	\
			cst_params.max_torque), c_torque_ctrl);
}

void set_torque_ctrl_param(ctrl_par &torque_ctrl_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_CTRL_PARAMETER);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kp_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kp_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Ki_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Ki_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kd_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kd_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Integral_limit);
}

void set_torque_ctrl_hall_param(hall_par &hall_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_CTRL_HALL);
	TORQUE_CTRL_WRITE(hall_params.pole_pairs);
	TORQUE_CTRL_WRITE(hall_params.max_ticks);
	TORQUE_CTRL_WRITE(hall_params.max_ticks_per_turn);
}

void set_torque_ctrl_qei_param(qei_par &qei_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_CTRL_QEI);
	TORQUE_CTRL_WRITE(qei_params.index);
	TORQUE_CTRL_WRITE(qei_params.real_counts);
	TORQUE_CTRL_WRITE(qei_params.max_ticks_per_turn);
	TORQUE_CTRL_WRITE(qei_params.poles);
	TORQUE_CTRL_WRITE(qei_params.max_ticks);
}
void set_torque_sensor(int sensor_used, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SENSOR_SELECT);
	TORQUE_CTRL_WRITE(sensor_used);
}

void enable_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(ENABLE_TORQUE_CTRL);
	TORQUE_CTRL_WRITE(1);
}

void shutdown_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SHUTDOWN_TORQUE_CTRL);
	TORQUE_CTRL_WRITE(0);
}

int check_torque_ctrl_state(chanend c_torque_ctrl)
{
	int state;
	TORQUE_CTRL_WRITE(TORQUE_CTRL_STATUS);
	TORQUE_CTRL_READ(state);
	return state;
}
