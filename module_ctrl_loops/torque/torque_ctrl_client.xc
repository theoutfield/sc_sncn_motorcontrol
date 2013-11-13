/**
 * \file torque_ctrl.xc
 *
 *	Torque control rountine based on field oriented Torque control method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "torque_ctrl_client.h"
#include <refclk.h>
#include <print.h>
#include <drive_config.h>
#include <internal_config.h>
#include <dc_motor_config.h>

int init_torque_control(chanend c_torque_ctrl)
{
	int init_state = INIT_BUSY;
	TORQUE_CTRL_ENABLE(); 					//signal torque ctrl loop

	// init check from torque control loop

	while(1)
	{
		init_state = __check_torque_init(c_torque_ctrl);
		//t when timerafter(time+2*MSEC_STD) :> time;
		if(init_state == INIT)
		{
//#ifdef debug_print
			//printstrln("torque intialized");
//#endif
			break;
		}
	}
	return init_state;
}

int get_torque(cst_par &cst_params, chanend c_torque_ctrl)
{
	int torque;
	TORQUE_CTRL_WRITE(GET_TORQUE_TOKEN);
	TORQUE_CTRL_READ(torque);
	return (torque); // *cst_params.motor_torque_constant too big to include
}

void set_torque(int torque,  cst_par &cst_params, chanend c_torque_ctrl)
{
	torque = torque; //cst_params.motor_torque_constant; too big to include
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
			cst_params.max_torque), cst_params , c_torque_ctrl);
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
	TORQUE_CTRL_WRITE(hall_params.gear_ratio);
	TORQUE_CTRL_WRITE(hall_params.pole_pairs);
}

void set_torque_ctrl_qei_param(qei_par &qei_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_CTRL_QEI);
	TORQUE_CTRL_WRITE(qei_params.gear_ratio);
	TORQUE_CTRL_WRITE(qei_params.index);
	TORQUE_CTRL_WRITE(qei_params.real_counts);
	TORQUE_CTRL_WRITE(qei_params.max_count);
	TORQUE_CTRL_WRITE(qei_params.poles);
}
void set_torque_sensor(int sensor_used, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SENSOR_SELECT);
	TORQUE_CTRL_WRITE(sensor_used);
}

void enable_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(ENABLE_TORQUE);
	TORQUE_CTRL_WRITE(0);
}

void shutdown_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SHUTDOWN_TORQUE);
	TORQUE_CTRL_WRITE(1);
}
