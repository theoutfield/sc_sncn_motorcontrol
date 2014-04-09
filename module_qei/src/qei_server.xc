
/**
 * \file qei_server.xc
 * \brief QEI Sensor Server Implementation
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

#include "qei_server.h"
#include <xscope.h>
//#pragma xta command "analyze loop qei_loop"
//#pragma xta command "set required - 1.0 us"
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

void qei_client_hanlder(chanend c_qei, int command, int position, int ok, int &count, int direction,\
		int init_state, int sync_out, int &calib_bw_flag, int &calib_fw_flag, int &offset_fw, \
		int &offset_bw, qei_par &qei_params, int &status)
{
	switch(command)
	{
		case QEI_RAW_POS_REQ:
			slave
			{
				c_qei <: position;
				c_qei <: ok;
			}
			break;

		case QEI_ABSOLUTE_POS_REQ:
			slave
			{
				c_qei <: count;
				c_qei <: direction;
			}
			break;

		case SYNC:
			slave
			{
				c_qei <: sync_out;
				c_qei <: calib_fw_flag;
				c_qei <: calib_bw_flag;
			}
			break;

		case SET_OFFSET:
			c_qei :> offset_fw;
			c_qei :> offset_bw;
			calib_bw_flag = 0;
			calib_fw_flag = 0;
			break;

		case CHECK_BUSY:
			c_qei <: init_state;
			break;

		case SET_QEI_PARAM_ECAT:
			c_qei :> qei_params.index;
			c_qei :> qei_params.max_ticks_per_turn;
			c_qei :> qei_params.real_counts;
			c_qei :> qei_params.poles;
			c_qei :> qei_params.max_ticks;
			c_qei :> qei_params.sensor_polarity;
			status = 1;
	//					printintln(qei_params.gear_ratio);
	//					printintln(qei_params.index);
	//					printintln(qei_params.max_ticks_per_turn);
	//					printintln(qei_params.real_counts);
			break;

		case QEI_RESET_COUNT:
			c_qei :> count;
			break;

		default:
			break;
	}
}

#pragma unsafe arrays
void run_qei(chanend c_qei_p1, chanend c_qei_p2, chanend c_qei_p3, chanend c_qei_p4, chanend c_qei_p5, \
		chanend c_qei_p6, port in p_qei, qei_par &qei_params)
{
	int position = 0;
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
	int max_count_actual = qei_params.max_ticks;
	int difference = 0;
	int direction = 0;
	int qei_max = qei_params.max_ticks_per_turn;
	int qei_type = qei_params.index;            // TODO use to disable sync for no-index
	int init_state = INIT;

	int qei_crossover = (qei_max*19)/100;
	int qei_count_per_hall = qei_params.real_counts / qei_params.poles;
	int offset_fw = 0;
	int offset_bw = 0;
	int calib_fw_flag = 0;
	int calib_bw_flag = 0;
	int sync_out = 0;
	int status = 0;
	int flag_index = 0;
	unsigned int new_pins_1;

	/*{
			xscope_register(4, XSCOPE_CONTINUOUS, "0 qei_position", XSCOPE_INT,	"n",
					           XSCOPE_CONTINUOUS, "1 qei_velocity", XSCOPE_INT,	"n",
					           XSCOPE_CONTINUOUS, "2 qei_position1", XSCOPE_INT, "n",
					           XSCOPE_CONTINUOUS, "3 qei_velocity1", XSCOPE_INT, "n");
			xscope_config_io(XSCOPE_IO_BASIC);
	}*/
	p_qei :> new_pins;

	while (1)
	{
		#pragma xta endpoint "qei_loop"
		#pragma ordered
		select
		{
			case p_qei when pinsneq(new_pins) :> new_pins :
				p_qei :> new_pins_1;
				p_qei :> new_pins_1;
				if(new_pins_1 == new_pins)
				{
					p_qei :> new_pins;
					if(new_pins_1 == new_pins)
					{
						v = lookup[new_pins][old_pins];

						if(qei_type == QEI_WITH_NO_INDEX)
						{
							{ v, position } = lmul(1, position, v, -5);
							if(position >= qei_params.real_counts )
								position = 0;
							else if(position <= -qei_params.real_counts )
								position = 0;
						}
						else
						{
							if (!v)
							{
								flag_index = 1;
								ok = 1;
								position = 0;
							}
							else
							{
								{ v, position } = lmul(1, position, v, -5);
								flag_index = 0;
							}
						}


					//	xscope_probe_data(0, position);

						old_pins = new_pins & 0x3;

						if(first == 1)
						{
							previous_position = position;
							first = 0;
						}

						if(previous_position != position )
						{
							difference = position - previous_position;
							//xscope_probe_data(1, difference);
							if( difference >= qei_crossover)
							{
								if(qei_params.sensor_polarity == NORMAL)
									count = count - 1;
								else
									count = count + 1;
								sync_out = offset_fw;  //valid needed
								calib_fw_flag = 1;
								direction = -1;
							}
							else if(difference <= -qei_crossover)
							{
								if(qei_params.sensor_polarity == NORMAL)
									count = count + 1;
								else
									count = count - 1;
								sync_out = offset_bw;
								calib_bw_flag = 1;
								direction = +1;
							}
							else if( difference <= 2 && difference > 0)
							{
								if(qei_params.sensor_polarity == NORMAL)
								{
									count = count + difference;
									sync_out = sync_out + difference;
								}
								else
								{
									count = count - difference;
									sync_out = sync_out - difference;
								}
								direction = -1;
							}
							else if( difference < 0 && difference >= -2)
							{
								if(qei_params.sensor_polarity == NORMAL)
								{
									count = count + difference;
									sync_out = sync_out + difference;
								}
								else
								{
									count = count - difference;
									sync_out = sync_out - difference;
								}
								direction = 1;
							}
							previous_position = position;
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
				}
				//xscope_probe_data(0, position);
				//xscope_probe_data(1, count);

				break;

			case c_qei_p1 :> command :
				qei_client_hanlder( c_qei_p1, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

			case c_qei_p2 :> command :
				qei_client_hanlder( c_qei_p2, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

			case c_qei_p3 :> command :
				qei_client_hanlder( c_qei_p3, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

			case c_qei_p4 :> command :
				qei_client_hanlder( c_qei_p4, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

			case c_qei_p5 :> command :
				qei_client_hanlder( c_qei_p5, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

			case c_qei_p6 :> command :
				qei_client_hanlder( c_qei_p6, command, position, ok, count, direction, init_state,\
						sync_out, calib_bw_flag, calib_fw_flag, offset_fw, offset_bw, qei_params,\
						status);
				break;

		}

		if(status == 1)
		{
			status = 0;
			max_count_actual = qei_params.max_ticks;
			qei_max = qei_params.max_ticks_per_turn;
			qei_type = qei_params.index;
			qei_crossover = (qei_max*19)/100;
			qei_count_per_hall = qei_params.real_counts / qei_params.poles;
		}
#pragma xta endpoint "qei_loop_end_point"
	}
}


