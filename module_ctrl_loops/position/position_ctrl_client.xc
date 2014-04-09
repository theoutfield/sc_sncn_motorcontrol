
/**
 * \file  position_ctrl_client.xc
 * \brief Position control Loop Client functions
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

#include <position_ctrl_client.h>
#include <print.h>
#include <drive_config.h>


//#define debug_print


int init_position_control(chanend c_position_ctrl)
{
	int ctrl_state = INIT_BUSY;

	while(1)
	{
		ctrl_state = check_position_ctrl_state(c_position_ctrl);
		if(ctrl_state == INIT_BUSY)
		{
			enable_position_ctrl(c_position_ctrl);
		}
		if(ctrl_state == INIT)
		{
			#ifdef debug_print
			printstrln("position control intialized");
			#endif
			break;
		}
	}
	return ctrl_state;
}

//internal functions
void set_position(int target_position, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SET_POSITION_TOKEN);
	POSITION_CTRL_WRITE(target_position);
}


int get_position(chanend c_position_ctrl)
{
	int position;
	POSITION_CTRL_WRITE(GET_POSITION_TOKEN);
	POSITION_CTRL_READ(position);
	return position;
}

int position_limit(int position, int max_position_limit, int min_position_limit)
{
	if (position > max_position_limit)
	{
		return max_position_limit;
	}
	else if (position < min_position_limit)
	{
		return min_position_limit;
	}
	else if (position >= min_position_limit && position <= max_position_limit)
	{
		return position;
	}
}

//csp mode function
void set_position_csp(csp_par &csp_params, int target_position, int position_offset, int velocity_offset,\
		              int torque_offset, chanend c_position_ctrl)
{
	set_position( position_limit( (target_position + position_offset) * csp_params.base.polarity ,	\
								   csp_params.max_position_limit,	\
								   csp_params.min_position_limit) , c_position_ctrl);
}


void set_position_ctrl_param(ctrl_par &position_ctrl_params, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SET_CTRL_PARAMETER);
	POSITION_CTRL_WRITE(position_ctrl_params.Kp_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Kp_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Ki_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Ki_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Kd_n);
	POSITION_CTRL_WRITE(position_ctrl_params.Kd_d);
	POSITION_CTRL_WRITE(position_ctrl_params.Integral_limit);
}

void set_position_ctrl_hall_param(hall_par &hall_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POSITION_CTRL_HALL;
	c_position_ctrl <: hall_params.pole_pairs;
}

void set_position_ctrl_qei_param(qei_par &qei_params, chanend c_position_ctrl)
{
	c_position_ctrl <: SET_POSITION_CTRL_QEI;
	c_position_ctrl <: qei_params.index;
	c_position_ctrl <: qei_params.real_counts;
	c_position_ctrl <: qei_params.max_ticks_per_turn;
}


void set_position_sensor(int sensor_used, chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SENSOR_SELECT);
	POSITION_CTRL_WRITE(sensor_used);
}

void enable_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(ENABLE_POSITION_CTRL);
	POSITION_CTRL_WRITE(1);
}

void shutdown_position_ctrl(chanend c_position_ctrl)
{
	POSITION_CTRL_WRITE(SHUTDOWN_POSITION_CTRL);
	POSITION_CTRL_WRITE(0);
}

int check_position_ctrl_state(chanend c_position_ctrl)
{
	int state;
	POSITION_CTRL_WRITE(POSITION_CTRL_STATUS);
	POSITION_CTRL_READ(state);
	return state;
}
