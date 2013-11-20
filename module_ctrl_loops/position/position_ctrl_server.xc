
/**
 *
 * \file position_ctrl_server.xc
 *
 *	Position Control Loop Server
 *
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
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
 * either expressed or implied, of the Synapticon GmbH Project.
 *
 */

#include <position_ctrl_server.h>
#include <xscope.h>
#include <print.h>
#include <drive_config.h>

//#define DEBUG
//#define debug_print

extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);


void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_position_ctrl, chanend c_commutation)
{
	int actual_position = 0;
	int target_position = 0;

	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;

	timer ts;
	unsigned int time;

	int command;
	int deactivate = 0;
	int activate = 0;
	int direction = 0;

	int precision;
	int precision_factor;

	int init_state = INIT_BUSY;

	while(1)
	{
		int received_command = UNSET;
		select
		{
			case POSITION_CTRL_READ(command):
				if(command == SET)
				{
					activate = SET;
					received_command = SET;
					while(1)
					{
						init_state = __check_commutation_init(c_commutation);
						if(init_state == INIT)
						{
							//printstrln("commutation intialized");
							init_state = INIT_BUSY;
							break;
						}
					}
#ifdef debug_print
					printstrln("pos activated");
#endif
				}
				else if(command == UNSET)
				{
					activate = UNSET;
					received_command = SET;
#ifdef debug_print
					printstrln("pos disabled");
#endif
				}
				else if(command == CHECK_BUSY)
				{
					POSITION_CTRL_WRITE(init_state);
				}
				break;

			default:
				break;
		}
		if(received_command == SET)
		{
//			POSITION_CTRL_WRITE(received_command);
			break;
		}
	}

	//printstrln("start pos");

	ts:> time;

	if(sensor_used == HALL)
	{
		precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_used);
		precision = HALL_PRECISION;
	}
	else if(sensor_used == QEI)
	{
		precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_used);
		precision = QEI_PRECISION;
	}

	init_state = INIT;
	while(activate)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + position_ctrl_params.Loop_time) :> time: // 1 ms

				/* acq actual position hall/qei */

				if(sensor_used == HALL)
				{   /* 100/(500*819) ~ 1/4095 appr (hall)  - to keep position info from hall in same range as qei*/
					{actual_position , direction}= get_hall_position_absolute(c_hall);
					actual_position = ( ( ( (actual_position/500)*precision_factor)/precision )/819)*100;
				}
				else if(sensor_used == QEI)
				{
					{actual_position, direction} =  get_qei_position_absolute(c_qei);
					actual_position = (actual_position * precision_factor)/precision;
				}

				/* Controller */

				error_position = (target_position - actual_position);
				error_position_I = error_position_I + error_position;
				error_position_D = error_position - previous_error;

				if(error_position_I > position_ctrl_params.Integral_limit)
				{
					error_position_I = position_ctrl_params.Integral_limit;
				}
				else if(error_position_I < -position_ctrl_params.Integral_limit)
				{
					error_position_I = 0 - position_ctrl_params.Integral_limit;
				}

				position_control_out = (position_ctrl_params.Kp_n * error_position)/position_ctrl_params.Kp_d   \
									 + (position_ctrl_params.Ki_n * error_position_I)/position_ctrl_params.Ki_d \
									 + (position_ctrl_params.Kd_n * error_position_D)/position_ctrl_params.Kd_d;

				if(position_control_out > position_ctrl_params.Control_limit)
				{
					position_control_out = position_ctrl_params.Control_limit;
				}
				else if(position_control_out < -position_ctrl_params.Control_limit)
				{
					position_control_out = 0 - position_ctrl_params.Control_limit;
				}

				if(!deactivate)
					set_commutation_sinusoidal(c_commutation, position_control_out);
				else
					set_commutation_sinusoidal(c_commutation, 0);

				#ifdef DEBUG
				xscope_probe_data(0, actual_position);
				#endif

				previous_error = error_position;

				break;

				/* acq target position from etherCAT */

			case POSITION_CTRL_READ(command):

				if(command == SET_POSITION_TOKEN)
				{
					POSITION_CTRL_READ(target_position);
				}
				else if(command == GET_POSITION_TOKEN)
				{
					POSITION_CTRL_WRITE(actual_position);
				}
				else if(command == CHECK_BUSY)
				{
					POSITION_CTRL_WRITE(init_state);
				}
				else if(command == SET_CTRL_PARAMETER)
				{
					POSITION_CTRL_READ(position_ctrl_params.Kp_n);
					POSITION_CTRL_READ(position_ctrl_params.Kp_d);
					POSITION_CTRL_READ(position_ctrl_params.Ki_n);
					POSITION_CTRL_READ(position_ctrl_params.Ki_d);
					POSITION_CTRL_READ(position_ctrl_params.Kd_n);
					POSITION_CTRL_READ(position_ctrl_params.Kd_d);
					POSITION_CTRL_READ(position_ctrl_params.Integral_limit);
				}
				else if(command == SENSOR_SELECT)
				{
					POSITION_CTRL_READ(sensor_used);
					if(sensor_used == HALL)
					{
						precision_factor = position_factor(hall_params.gear_ratio, 1, hall_params.pole_pairs, sensor_used);
						precision = HALL_PRECISION;
					}
					else if(sensor_used == QEI)
					{
						precision_factor = position_factor(qei_params.gear_ratio, qei_params.real_counts, 1, sensor_used);
						precision = QEI_PRECISION;
					}
				}
				else if(command == SHUTDOWN_POSITION)
					POSITION_CTRL_READ(deactivate);

				else if(command == ENABLE_POSITION)
					POSITION_CTRL_READ(deactivate);

				else if(command == SET_POSITION_CTRL_HALL)
				{
					c_position_ctrl :> hall_params.gear_ratio;
					c_position_ctrl :> hall_params.pole_pairs;
				}
				else if(command == SET_POSITION_CTRL_QEI)
				{
					c_position_ctrl :> qei_params.gear_ratio;
					c_position_ctrl :> qei_params.index;
					c_position_ctrl :> qei_params.real_counts;
					c_position_ctrl :> qei_params.max_count;
				}
				break;
		}

	}
}
