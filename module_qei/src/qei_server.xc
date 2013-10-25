/*
 * File:    qei_server.xc
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

#include "qei_server.h"

#include <xscope.h>

// Order is 00 -> 10 -> 11 -> 01
// Bit 3 = Index
static const unsigned char lookup[16][4] = {
		{ 5, 4, 6, 5 }, // 00 00
		{ 6, 5, 5, 4 }, // 00 01
		{ 4, 5, 5, 6 }, // 00 10
		{ 5, 6, 4, 5 }, // 00 11
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx
		{ 0, 0, 0, 0 }, // 01 xx

		{ 5, 4, 6, 5 }, // 10 00
		{ 6, 5, 5, 4 }, // 10 01
		{ 4, 5, 5, 6 }, // 10 10
		{ 5, 6, 4, 5 }, // 10 11
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }, // 11 xx
		{ 0, 0, 0, 0 }  // 11 xx
};

void qei_client_hanlder(chanend c_qei, int command, int position, int ok, int count, int direction,\
		int init_state, int sync_out, int &calib_bw_flag, int &calib_fw_flag, int &offset_fw, int &offset_bw)
{
	if(command == QEI_RAW_POS_REQ)
	{
		slave
		{
			c_qei <: position;
			c_qei <: ok;
		}
	}
	else if(command == QEI_ABSOLUTE_POS_REQ)
	{
		slave
		{
			c_qei <: count;
			c_qei <: direction;
		}
	}
	else if(command == SYNC)
	{
		slave
		{
			c_qei <: sync_out;
			c_qei <: calib_fw_flag;
			c_qei <: calib_bw_flag;
		}
	}
	else if(command == SET_OFFSET)
	{
		c_qei :> offset_fw;
		c_qei :> offset_bw;
		calib_bw_flag = 0;
		calib_fw_flag = 0;
	}
	/*	else if(command == QEI_VELOCITY_PWM_RES_REQ)
	{
		slave
		{
			c_qei <: velocity_raw1;
		}
	}*/
	else if(command == CHECK_BUSY)
	{
		c_qei <: init_state;
	}
}

#pragma unsafe arrays
void run_qei(chanend c_qei_p1, chanend c_qei_p2, chanend c_qei_p3, chanend c_qei_p4, chanend c_qei_p5, port in p_qei, qei_par &qei_params)
{
	unsigned int position = 0;
	unsigned int v;
	unsigned int ts1;
	unsigned int ts2;
	unsigned int ok = 0;
	unsigned int old_pins = 0;
	unsigned int new_pins;

	int command;
	int current_pos = 0;
	int previous_position = 0;
	int count = 0;
	int first = 1;
	int max_count_actual = qei_params.gear_ratio * qei_params.real_counts;
	int difference = 0;
	int direction = 0;
	int qei_max = qei_params.max_count;
	int qei_type = qei_params.index;
	int init_state = INIT;

	int qei_crossover = qei_max - qei_max/10;
	int qei_count_per_hall = qei_params.real_counts / qei_params.poles;
	int offset_fw = 185;
	int offset_bw = 104;
	int calib_fw_flag = 0;
	int calib_bw_flag = 0;
	int sync_out = 0;


	p_qei :> new_pins;

	while (1) {
	#pragma ordered
		select {
			case p_qei when pinsneq(new_pins) :> new_pins :
				{
				  	  if(qei_type == QEI_WITH_INDEX)
				  	  {
						  v = lookup[new_pins][old_pins];

						  if (!v) {
							  position = 0;
							  ok = 1;
						  }
						  else
						  {
							  { v, position } = lmul(1, position, v, -5);
						  }
				  	  }
				  	  else if(qei_type == QEI_WITH_NO_INDEX)
				  	  {
				  		  v = lookup[new_pins][old_pins];
				  		  { v, position } = lmul(1, position, v, -5);
				  	  }

				  	  old_pins = new_pins & 0x3;

				  	if(first == 1)
					{
						previous_position = position & (qei_max-1);
						first = 0;
					}
					current_pos =  position & (qei_max-1);
					if(previous_position != current_pos )
					{
						difference = current_pos - previous_position;
						if( difference > qei_crossover)
						{
							count = count + 1;
							sync_out = offset_fw;
							calib_fw_flag = 1;
							direction = 1;
						}
						else if(difference < -qei_crossover)
						{
							count = count - 1;
							sync_out = offset_bw;
							calib_bw_flag = 1;
							direction = -1;
						}
						else if( difference < 10 && difference >0)
						{
							count = count - difference;
							sync_out = sync_out - difference;
							direction = -1;
						}
						else if( difference < 0 && difference > -10)
						{
							count = count - difference;
							sync_out = sync_out - difference;
							direction = 1;
						}
						previous_position = current_pos;
					}
					if(sync_out < 0)
					{
						sync_out = qei_count_per_hall + sync_out;
					}
					if(count >= max_count_actual || count <= -max_count_actual)
					{
						count=0;
					}
					if(sync_out >= qei_count_per_hall )
					{
						sync_out = 0;
					}
				}
				break;

			case c_qei_p1 :> command :
				qei_client_hanlder( c_qei_p1, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw);
				break;

			case c_qei_p2 :> command :
				qei_client_hanlder( c_qei_p2, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw);
				break;

			case c_qei_p3 :> command :
				qei_client_hanlder( c_qei_p3, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw);
				break;

			case c_qei_p4 :> command :
				qei_client_hanlder( c_qei_p4, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw);
				break;

			case c_qei_p5 :> command :
				qei_client_hanlder( c_qei_p5, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw);
				break;

		}

	}
}


