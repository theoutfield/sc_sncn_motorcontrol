
/**
 * \file comm_loop_client.xc
 *
 *	Commutation rountine based on Space Vector PWM method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			& Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/
#include "comm_loop_client.h"
#include <xs1.h>
#include <stdint.h>
#include <bldc_motor_config.h>
#include "refclk.h"
#include <internal_config.h>
#include "print.h"


void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed)
{
	commutation_params.angle_variance = 1024/(hall_params.pole_pairs * 3); // (60 * 4096)/( POLE_PAIRS * 2 *360)
	if(hall_params.pole_pairs < 4)
	{
		commutation_params.max_speed_reached = nominal_speed*4;
		commutation_params.flag = 1;
	}
	else if(hall_params.pole_pairs >=4)
	{
		commutation_params.max_speed_reached = nominal_speed;//10000;//
		commutation_params.flag = 0;
	}
	commutation_params.qei_forward_offset = 0;
	commutation_params.qei_backward_offset = 0;
//	printintln(commutation_params.angle_variance);
//	printintln(commutation_params.max_speed_reached);
}


void commutation_sensor_select(chanend c_commutation, int sensor_select)
{
	c_commutation <: SENSOR_SELECT;
	c_commutation <: sensor_select;
	return;
}

void set_commutation_params(chanend c_commutation, commutation_par &commutation_params)
{
	c_commutation <: SET_COMMUTATION_PARAMS;
	c_commutation <: commutation_params.angle_variance;
	c_commutation <: commutation_params.max_speed_reached;
	c_commutation <: commutation_params.offset_forward;
	c_commutation <: commutation_params.offset_backward;
}

/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage)
{
	c_commutation <: 2;
	c_commutation <: input_voltage;
	return;
}

