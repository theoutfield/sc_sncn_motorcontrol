/*
 *
 * File:    qei_client.h
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
#ifndef __QEI_CLIENT_H__
#define __QEI_CLIENT_H__

#include<dc_motor_config.h>
#include "filter_blocks.h"
#include <print.h>
#include <xs1.h>
#include <stdio.h>
#include "qei_config.h"

typedef struct QEI_VELOCITY_PARAM
{
	int previous_position;
	int old_difference;
	int filter_buffer[8];
	int index;
	int filter_length;
} qei_velocity_par;

void init_qei_velocity_params(qei_velocity_par &qei_velocity_params);

///only position and valid
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params);

//counted up position and direction
{int, int} get_qei_position_absolute(chanend c_qei);


int qei_speed(chanend c_qei, qei_par &qei_params, qei_velocity_par &qei_velocity_params);

//return velocity
int get_qei_velocity(chanend c_qei, qei_par &qei_params);

int _get_qei_velocity_pwm_resolution(chanend c_qei, qei_par &qei_params);

extern int __qei_max_counts(int real_counts);
int get_qei_syncp(chanend c_qei);
#endif /* __QEI_CLIENT_H__ */
