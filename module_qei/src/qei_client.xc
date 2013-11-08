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


#include "qei_client.h"

void init_qei_velocity_params(qei_velocity_par &qei_velocity_params)
{
	qei_velocity_params.previous_position=0;
	qei_velocity_params.old_difference = 0;
	qei_velocity_params.filter_length =8;
	qei_velocity_params.index = 0;
	init_filter(qei_velocity_params.filter_buffer, qei_velocity_params.index, qei_velocity_params.filter_length);
	return;
}

//get position and valid from qei directly
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params)
{
	unsigned int position;
	unsigned int valid;

	c_qei <: QEI_RAW_POS_REQ;
	master
	{
		c_qei :> position;
		c_qei :> valid;
	}
	position &= (qei_params.max_count - 1);

	return {position, valid};
}

//counted up position from qei with gear ratio
{int, int} get_qei_position_absolute(chanend c_qei)
{
	int position;
	int direction; 				// clockwise +1  counterclockwise -1
	c_qei <: QEI_ABSOLUTE_POS_REQ;
	master
	{
		c_qei :> position;
		c_qei :> direction;
	}
	return {position, direction};
}

int get_qei_velocity(chanend c_qei, qei_par &qei_params, qei_velocity_par &qei_velocity_params)
{
	int difference;
	int count;
	int direction;
	int qei_crossover = qei_params.max_count - qei_params.max_count/10;
	{count, direction} = get_qei_position_absolute(c_qei);
	difference = count - qei_velocity_params.previous_position;
	if(difference > qei_crossover)
		difference = qei_velocity_params.old_difference;
	else if(difference < -qei_crossover)
		difference = qei_velocity_params.old_difference;
	qei_velocity_params.previous_position = count;
	qei_velocity_params.old_difference = difference;
	return (filter(qei_velocity_params.filter_buffer, qei_velocity_params.index, qei_velocity_params.filter_length, difference)*1000*60) / (qei_params.real_counts);
}

{int, int, int} get_qei_sync_position(chanend c_qei)
{
	int position, calib_fw_flag, calib_bw_flag;
	c_qei <: SYNC;
	master
	{
		c_qei :> position;
		c_qei :> calib_fw_flag;
		c_qei :> calib_bw_flag;
	}
	return {position , calib_fw_flag, calib_bw_flag};
}

void set_qei_sync_offset(chanend c_qei, int offset_forward, int offset_backward)
{
	c_qei <: SET_OFFSET;
	c_qei <: offset_forward;
	c_qei <: offset_backward;
	return;
}


