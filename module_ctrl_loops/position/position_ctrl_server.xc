
/**
 * \file  position_ctrl_server.xc
 * \brief Position Control Loop Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
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

#include <position_ctrl_server.h>
#include "brushed_dc_client.h"
#include <xscope.h>
#include <print.h>
#include <drive_config.h>

//#define DEBUG
//#define debug_print

//extern int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used);


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

	int command = 0;
	int deactivate = 0;
	int activate = 0;
	int direction = 0;

	int fet_state = 0;
	int init_state = INIT_BUSY; /* check commutation init */

	#ifdef DEBUG
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_position", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_position", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	#endif
	//printstrln("start pos");

	if(sensor_used == HALL)
	{
		{actual_position, direction} = get_hall_position_absolute(c_hall);
		target_position = actual_position;
	//	printintln(target_position);
	//	printintln(actual_position);
	}
	else if(sensor_used == QEI)
	{
		{actual_position, direction} = get_qei_position_absolute(c_qei);
		target_position = actual_position;
	//	printintln(target_position);
	//	printintln(actual_position);
	}
	/**
	 * Or any other sensor interfaced to the IFM Module
	 * place client functions here to acquire position
	 */

	ts:> time;

	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + position_ctrl_params.Loop_time) :> time: // 1 ms

				if(activate == 1)
				{
					/* acquire actual position hall/qei/sensor */
					switch(sensor_used)
					{
						case HALL:
							{actual_position , direction} = get_hall_position_absolute(c_hall);
							break;

						case QEI:
							{actual_position, direction} =  get_qei_position_absolute(c_qei);
							break;

						/**
						 * Or any other sensor interfaced to the IFM Module
						 * place client functions here to acquire position
						 */
					}

					/* PID Controller */

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

				#ifndef BDC
					set_commutation_sinusoidal(c_commutation, position_control_out);
				#else
					set_bdc_voltage(c_commutation, position_control_out);
				#endif

					#ifdef DEBUG
						xscope_probe_data(0, actual_position);
						xscope_probe_data(1, target_position);
					#endif
						//xscope_probe_data(2, target_position);
					previous_error = error_position;
				}

				break;

			case POSITION_CTRL_READ(command):
				switch(command)
				{
					case SET_POSITION_TOKEN:
						POSITION_CTRL_READ(target_position);
						break;

					case GET_POSITION_TOKEN:
						POSITION_CTRL_WRITE(actual_position);
						break;

					case CHECK_BUSY: 					/* Check init state */
						POSITION_CTRL_WRITE(activate);
						break;

					case SET_CTRL_PARAMETER:
						POSITION_CTRL_READ(position_ctrl_params.Kp_n);
						POSITION_CTRL_READ(position_ctrl_params.Kp_d);
						POSITION_CTRL_READ(position_ctrl_params.Ki_n);
						POSITION_CTRL_READ(position_ctrl_params.Ki_d);
						POSITION_CTRL_READ(position_ctrl_params.Kd_n);
						POSITION_CTRL_READ(position_ctrl_params.Kd_d);
						POSITION_CTRL_READ(position_ctrl_params.Integral_limit);
						break;

					case SET_POSITION_CTRL_HALL:
						c_position_ctrl :> hall_params.pole_pairs;
						break;

					case SET_POSITION_CTRL_QEI:
						c_position_ctrl :> qei_params.index;
						c_position_ctrl :> qei_params.real_counts;
						c_position_ctrl :> qei_params.max_ticks_per_turn;
						break;

					case SENSOR_SELECT:
						POSITION_CTRL_READ(sensor_used);
						if(sensor_used == HALL)
						{
							{actual_position , direction}= get_hall_position_absolute(c_hall);
						}
						else if(sensor_used == QEI)
						{
							{actual_position, direction} = get_qei_position_absolute(c_qei);
						}
						/**
						 * Or any other sensor interfaced to the IFM Module
						 * place client functions here to acquire position
						 */
						target_position = actual_position;
						break;

					case ENABLE_POSITION_CTRL:
						POSITION_CTRL_READ(activate);
						activate = SET;
						while(1)
						{
							init_state = __check_commutation_init(c_commutation);
							if(init_state == INIT)
							{
								#ifdef debug_print
								printstrln("commutation intialized");
								#endif
								fet_state = check_fet_state(c_commutation);
								if(fet_state == 1)
								{
									enable_motor(c_commutation);
									wait_ms(2, 1, ts);
								}
								break;
							}
						}
						#ifdef debug_print
						printstrln("position control activated");
						#endif
						break;

					case SHUTDOWN_POSITION_CTRL:	// SHUTDOWN_POSITION_CTRL
						POSITION_CTRL_READ(activate);
						set_commutation_sinusoidal(c_commutation, 0);
						error_position = 0;
						error_position_D = 0;
						error_position_I = 0;
						previous_error = 0;
						position_control_out = 0;
						//	target_position = 0;
						disable_motor(c_commutation);
						wait_ms(30, 1, ts); //
						#ifdef debug_print
							printstrln("position control disabled");
						#endif
						break;

					case POSITION_CTRL_STATUS: //check active state
						POSITION_CTRL_WRITE(activate);
						break;

					default:
						break;
				}
				break;
		}

	}
}
