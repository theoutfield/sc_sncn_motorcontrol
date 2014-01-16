
/**
 *
 * \file velocity_ctrl_server.xc
 *
 *	Velocity Control Loop Server
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
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "velocity_ctrl_server.h"
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop_client.h"
#include "filter_blocks.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"

//#define Debug_velocity_ctrl
#define debug_print

void velocity_control(ctrl_par &velocity_ctrl_params, filter_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params, \
	 	 	 int sensor_used, chanend c_hall, chanend c_qei, chanend c_velocity_ctrl, chanend c_commutation)
{
	/* Controller declarations */
	int actual_velocity = 0;
	int target_velocity = 0;
	int error_velocity = 0;
	int error_velocity_D = 0;
	int error_velocity_I = 0;
	int previous_error = 0;
	int velocity_control_out = 0;

	timer ts;
	unsigned int time;

	/* Sensor filter declarations */
	int filter_length = sensor_filter_params.filter_length;
	int filter_buffer[FILTER_SIZE_MAX];						//default size used at compile time (cant be changed further)
	int index = 0;

	/* speed calc declarations */
	int position;
	int init = 0;
	int previous_position = 0;
	int raw_speed = 0;			// rpm
	int difference;
	int direction = 0;
	int old_difference;
	int rpm_constant = 1000*60; // constant
	int speed_factor_hall = hall_params.pole_pairs*4095*(velocity_ctrl_params.Loop_time/MSEC_STD); 		// variable pole_pairs    core 2/1/0 only
	int speed_factor_qei  = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);		  	// variable qei_real_max  core 2/1/0 only

	int command;
	int deactivate = 0;
	int activate = 0;
	int init_state = INIT_BUSY;
	int qei_crossover = qei_params.max_count - qei_params.max_count/10;
	int hall_crossover = (hall_params.pole_pairs * hall_params.gear_ratio * 4095 * 9 )/10;

	init_filter(filter_buffer, index, FILTER_SIZE_MAX);
	while(1)
	{
		int received_command = UNSET;
#pragma ordered
		select
		{
			case VELOCITY_CTRL_READ(command):
				if(command == ENABLE_VELOCITY_CTRL)
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
					printstrln("vel activated");
#endif
				}
				else if(command == SHUTDOWN_VELOCITY_CTRL)
				{
					activate = UNSET;
					received_command = SET;
#ifdef debug_print
					printstrln("vel disabled");
#endif
				}
				else if(command == CHECK_BUSY)
				{
					VELOCITY_CTRL_WRITE(activate);
				}
				else if(command == VELOCITY_CTRL_STATUS)
				{
					VELOCITY_CTRL_WRITE(activate);
				}
				break;
			default:
				break;
		}
		if(received_command == SET)
		{
			break;
		}
	}



	//printstrln("start vel");
	ts :> time;
	//ts when timerafter(time+1*SEC_FAST) :> time;

	init_state = INIT;
	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + velocity_ctrl_params.Loop_time) :> time:

				if(activate == 1)
				{
					/* calculate actual velocity from hall/qei with filter*/
					if(sensor_used == HALL)
					{
						if(init == 0)
						{
							//set_commutation_sinusoidal(c_commutation, 400);
							{position, direction} = get_hall_position_absolute(c_hall);
							if(position > 2049)
							{
								init = 1;
								previous_position = 2049;
							}
							else if(position < -2049)
							{
								init = 1;
								previous_position = -2049;
							}
							raw_speed = 0;

							//target_velocity = 0;
						}
						else if(init == 1)
						{
							{position, direction} = get_hall_position_absolute(c_hall);
							difference = position - previous_position;
							if(difference > hall_crossover)
								difference = old_difference;
							else if(difference < -hall_crossover)
								difference = old_difference;
							raw_speed = (difference*rpm_constant)/speed_factor_hall;
			#ifdef Debug_velocity_ctrl
							xscope_probe_data(0, raw_speed);
			#endif
							previous_position = position;
							old_difference = difference;
						}
					}
					else if(sensor_used == QEI)
					{
						{position, direction} = get_qei_position_absolute(c_qei);
						difference = position - previous_position;
						if(difference > qei_crossover)
							difference = old_difference;
						if(difference < -qei_crossover)
							difference = old_difference;
						raw_speed = (difference*rpm_constant)/speed_factor_qei;

			#ifdef Debug_velocity_ctrl
						xscope_probe_data(0, raw_speed);
			#endif

						previous_position = position;
						old_difference = difference;
					}



					actual_velocity = filter(filter_buffer, index, filter_length, raw_speed);


			#ifdef Debug_velocity_ctrl
					xscope_probe_data(1, actual_velocity);
			#endif

					/* Controller */
					error_velocity   = (target_velocity - actual_velocity);
					error_velocity_I = error_velocity_I + error_velocity;
					error_velocity_D = error_velocity - previous_error;

					if(error_velocity_I > (velocity_ctrl_params.Integral_limit))
						error_velocity_I = (velocity_ctrl_params.Integral_limit);
					else if(error_velocity_I < -(velocity_ctrl_params.Integral_limit))
						error_velocity_I = 0 -(velocity_ctrl_params.Integral_limit);

					velocity_control_out = (velocity_ctrl_params.Kp_n*error_velocity)/(velocity_ctrl_params.Kp_d)   \
										 + (velocity_ctrl_params.Ki_n*error_velocity_I)/(velocity_ctrl_params.Ki_d) \
										 + (velocity_ctrl_params.Kd_n*error_velocity_D)/(velocity_ctrl_params.Kd_d);

					if(velocity_control_out > velocity_ctrl_params.Control_limit)
						velocity_control_out = velocity_ctrl_params.Control_limit;
					else if(velocity_control_out < -velocity_ctrl_params.Control_limit)
						velocity_control_out = 0 - velocity_ctrl_params.Control_limit;


					set_commutation_sinusoidal(c_commutation, velocity_control_out);
					/*if(!deactivate)
						set_commutation_sinusoidal(c_commutation, velocity_control_out);
					else
						set_commutation_sinusoidal(c_commutation, 0);*/

					previous_error = error_velocity;

				}
				break;

				/* acq target velocity etherCAT */
			case VELOCITY_CTRL_READ(command):

				if(command == SET_VELOCITY_TOKEN)
					VELOCITY_CTRL_READ(target_velocity);

				else if(command == GET_VELOCITY_TOKEN)
					VELOCITY_CTRL_WRITE(actual_velocity);

				else if(command == CHECK_BUSY)
					VELOCITY_CTRL_WRITE(activate);

				else if(command == SET_CTRL_PARAMETER)
				{
					VELOCITY_CTRL_READ(velocity_ctrl_params.Kp_n);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Kp_d);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Ki_n);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Ki_d);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Kd_n);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Kd_d);
					VELOCITY_CTRL_READ(velocity_ctrl_params.Integral_limit);
				}
				else if(command == SENSOR_SELECT)
				{
					VELOCITY_CTRL_READ(sensor_used);
					if(sensor_used == HALL)
					{
						speed_factor_hall = hall_params.pole_pairs*4095*(velocity_ctrl_params.Loop_time/MSEC_STD);
					}
					else if(sensor_used == QEI)
					{
						speed_factor_qei = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);
						qei_crossover = qei_params.max_count - qei_params.max_count/10;
					}
				}


				else if(command == SHUTDOWN_VELOCITY_CTRL)
				{
					VELOCITY_CTRL_READ(activate);
					set_commutation_sinusoidal(c_commutation, 0);
					error_velocity = 0;
					error_velocity_D = 0;
					error_velocity_I = 0;
					previous_error = 0;
					velocity_control_out = 0;
					index = 0;
					previous_position = 0;
					raw_speed = 0;
					old_difference = 0;
					init = 0;
					init_filter(filter_buffer, index, FILTER_SIZE_MAX);
				}
				else if(command == ENABLE_VELOCITY_CTRL)
				{
					VELOCITY_CTRL_READ(activate);
					activate = SET;
					while(1)
					{
						init_state = __check_commutation_init(c_commutation);
						if(init_state == INIT)
						{
							printstrln("commutation intialized");
							break;
						}
					}
					#ifdef debug_print
						printstrln("vel activated");
					#endif
				}
				else if(command == VELOCITY_CTRL_STATUS)
				{
					VELOCITY_CTRL_WRITE(activate);
				}

				else if(command == SET_VELOCITY_CTRL_HALL)
				{
					VELOCITY_CTRL_READ(hall_params.gear_ratio);
					VELOCITY_CTRL_READ(hall_params.pole_pairs);
					speed_factor_hall = hall_params.pole_pairs*4095*(velocity_ctrl_params.Loop_time/MSEC_STD);
				}
				else if(command == SET_VELOCITY_CTRL_QEI)
				{
					VELOCITY_CTRL_READ(qei_params.gear_ratio);
					VELOCITY_CTRL_READ(qei_params.index);
					VELOCITY_CTRL_READ(qei_params.real_counts);
					VELOCITY_CTRL_READ(qei_params.max_count);
					VELOCITY_CTRL_READ(qei_params.poles);
					speed_factor_qei = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);
					qei_crossover = qei_params.max_count - qei_params.max_count/10;
				}
				else if(command == SET_VELOCITY_FILTER)
				{
					VELOCITY_CTRL_READ(filter_length);
					if(filter_length > FILTER_SIZE_MAX)
						filter_length = FILTER_SIZE_MAX;
				}
				break;

		}



	}


}
