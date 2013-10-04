/*
 *
 * File:    qei_client.xc
 *
 * Get the position from the QEI server
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 */

#include <xs1.h>
#include <stdio.h>
#include "qei_config.h"
#include "qei_client.h"
#include <dc_motor_config.h>
#include <print.h>

//get position and valid from qei directly
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params)
{
	unsigned int p, v;


	c_qei <: QEI_RAW_POS_REQ;
	master {
		c_qei :> p;
		c_qei :> v;
	}
	p &= (qei_params.max_count - 1);

	return {p, v};
}

//counted up position from qei with gear ratio
{int, int} get_qei_position_absolute(chanend c_qei)
{
	int pos;
	int dirn; 				// clockwise +1  counterclockwise -1
	c_qei <: QEI_ABSOLUTE_POS_REQ;
	master
	{
		c_qei :> pos;
		c_qei :> dirn;
	}
	return {pos, dirn};
}

int get_qei_velocity(chanend c_qei, qei_par &qei_params)
{
	int velocity;

	c_qei <: QEI_VELOCITY_REQ;
	master
	{
		c_qei :> velocity;
	}
	velocity = ((velocity / FILTER_LENGTH_QEI)*QEI_RPM_CONST) / (qei_params.real_counts);

	return velocity;
}


int _get_qei_velocity_pwm_resolution(chanend c_qei, qei_par &qei_params)
{
	int velocity;

	c_qei <: QEI_VELOCITY_PWM_RES_REQ;
	master
	{
		c_qei :> velocity;
	}
	velocity = ((velocity/FILTER_LENGTH_QEI_PWM)*QEI_PWM_RPM_CONST) / (qei_params.real_counts);

	return velocity;
}
