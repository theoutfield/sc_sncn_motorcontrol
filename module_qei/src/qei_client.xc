
/**
 * \file qei_client.xc
 * \brief QEI Sensor Client Functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH & XMOS Ltd
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
	position &= (qei_params.max_ticks_per_turn - 1);

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
	int qei_crossover = qei_params.real_counts - qei_params.real_counts/10;
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

void reset_qei_count(chanend c_qei, int offset)
{
	c_qei <: QEI_RESET_COUNT;
	c_qei <: offset;
}
