
/**
 *
 * \file test_velocity-ctrl.xc
 *
 * \brief Main project file
 *  Test illustrates usage of profile velocity control
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
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

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <ioports.h>
#include <hall_server.h>
#include <qei_server.h>
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <velocity_ctrl_server.h>
#include <xscope.h>
#include <profile.h>
#include <internal_config.h>
#include <bldc_motor_config.h>
#include <drive_config.h>
#include <profile_control.h>
#include "velocity_ctrl_client.h"
#include <flash_somanet.h>
#include <qei_client.h>
//#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


void xscope_initialise_1()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}


/* Test Profile Velocity function */
void profile_velocity_test(chanend c_velocity_ctrl)
{
	int target_velocity = 1000;	 		// rpm
	int acceleration 	= 1000;			// rpm/s
	int deceleration 	= 1000;			// rpm/s

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);

	target_velocity = 0;				// rpm
	set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);

	/*target_velocity = -4000;				// rpm
		set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);

		target_velocity = 0;				// rpm
		set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);*/
}

{int, int, int, int} get_home_state(chanend c_home)
{
	int position;
	int direction;
	int home_state;
	int safety_state;
	c_home <: 1;
	c_home :> home_state;
	c_home :> safety_state;
	c_home :> position;
	c_home :> direction;
	return {home_state, safety_state, position, direction};
}
void home_track(port in p_ifm_ext_d0, port in p_ifm_ext_d1, chanend c_home, chanend c_qei, chanend c_hall)
{
	timer t;
	unsigned int time, time1;
	int home_switch;
	int home_switch_1;
	int home_state = 0;
	int safety_state = 0;
	int safety_switch;
	int safety_switch_1;
	int position = 0;
	int direction;
	int command;

	int switch_type = 1; //active high switch = 1/ active low switch = 2
	if(switch_type == 1)
	{
		set_port_pull_down(p_ifm_ext_d0);
		set_port_pull_down(p_ifm_ext_d1);
	}
	t:>time;
	printstrln("start");
	xscope_initialise_1();
	p_ifm_ext_d0 :> home_switch;   //once
	p_ifm_ext_d1 :> safety_switch; //once
	if(home_switch == 1)
	{
		//printstrln("home");
		home_state = 1;
		xscope_probe_data(0, home_state);
	}
	while(1)
	{

		//xscope_probe_data(0, home_switch);
		//xscope_probe_data(1, safety_switch);


		select
		{
			case p_ifm_ext_d0 when pinsneq(home_switch) :> home_switch:
				//t when timerafter(time + 83333) :> time; //3khz
				if(home_switch == 1)
				{//register pos data
					t :> time1;
					t when timerafter(time1 + 83333) :> time1;
					p_ifm_ext_d0 :> home_switch_1;
					if(home_switch_1 == 1)
					{
						t when timerafter(time1 + 83333) :> time1;
						p_ifm_ext_d0 :> home_switch_1;
						if(home_switch_1 == 1)
						{
							home_state = 1;
							xscope_probe_data(0, home_state);
							{position, direction} = get_qei_position_absolute(c_qei);
						//printstrln("home");//xscope_probe_data(0, home_switch);  // confirm home position
						}
					}
				}
				else
				{
					home_state = 0;
					xscope_probe_data(0, home_state);
				}
			//case p_ifm_ext_d0 :> home_switch:
				//if(home_switch == 1) // high  ON
				//	printstrln("home");
				break;

			case p_ifm_ext_d1 when pinsneq(safety_switch) :> safety_switch:
			//case p_ifm_ext_d1 :> safety_switch:  // find safety switch in reverse direction
				//if(safety_switch == 1) // low OFF
				//	printstrln("safety");

				if(safety_switch == 1)
				{//register pos data
					t :> time1;
					t when timerafter(time1 + 83333) :> time1;
					p_ifm_ext_d1 :> safety_switch_1;
					if(safety_switch_1 == 1)
					{
						t when timerafter(time1 + 83333) :> time1;
						p_ifm_ext_d1 :> safety_switch_1;
						if(safety_switch_1 == 1)
						{
							safety_state = 1;
							//xscope_probe_data(0, safety_state);
							//{position, direction} = get_qei_position_absolute(c_qei);
						//printstrln("home");//xscope_probe_data(0, home_switch);  // confirm home position
						}
					}
				}
				else
				{
					safety_state = 0;
				}
				break;


			case c_home :> command:   //case upon request send home state with position info too
				c_home <: home_state;
				c_home <: safety_state;
				c_home <: position;
				c_home <: direction;
				break;
		}
	}
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6;					// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;				// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl, c_adctrig;												// pwm channels
	chan c_velocity_ctrl;													// velocity control channel
	chan c_watchdog; 														// watchdog channel

	// EtherCat Comm channels
	chan coe_in; 		// CAN from module_ethercat to consumer
	chan coe_out; 		// CAN from consumer to module_ethercat
	chan eoe_in; 		// Ethernet from module_ethercat to consumer
	chan eoe_out; 		// Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 		// File from module_ethercat to consumer
	chan foe_out; 		// File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;
	chan c_sig_1;
 chan c_home;

	par
	{
		on stdcore[3]:
		{
			home_track( p_ifm_ext_d0, p_ifm_ext_d1, c_home, c_qei_p5, c_hall_p5);
		}


		/* Test Profile Velocity function */
		on stdcore[1]:
		{
			{
				int target_velocity = 2000;
				int acceleration = 3000;
				int deceleration = 3000;
				int max_profile_velocity = 4000;
				int reset_counter = 0;
				int velocity_ramp;
				int actual_velocity;
				int direction;
				int i = 1;
				timer t;
				unsigned int time;
				int steps;
				int limit_switch = -1; // positive negative limit switches
				int home_state = 0;
				int safety_state = 0;
				int position = 0;
				int current_position = 0;
				int offset = 0;
				int dirn;
				int end_state = 0;
				int init_state = __check_velocity_init(c_velocity_ctrl);
				while(1)
				{
					init_state = init_velocity_control(c_velocity_ctrl);
					if(init_state == INIT)
						break;
				}

				i = 1;
				actual_velocity = get_velocity(c_velocity_ctrl);
				steps = init_velocity_profile(target_velocity*limit_switch, actual_velocity, acceleration, deceleration, max_profile_velocity);
				t :> time;
				while(1)
				{
					select
					{

						case t when timerafter(time + MSEC_STD) :> time:
							if(i < steps)
							{
								velocity_ramp = velocity_profile_generate(i);
								i = i+1;
							}
							set_velocity(velocity_ramp, c_velocity_ctrl);
							{home_state, safety_state, position, direction} = get_home_state(c_home);
							if(home_state == 1 || safety_state == 1)
							{
								actual_velocity = get_velocity(c_velocity_ctrl);
								steps = init_velocity_profile(0, actual_velocity, acceleration, deceleration, max_profile_velocity);
								for(i = 1; i < steps; i++)
								{
									velocity_ramp = velocity_profile_generate(i);
									set_velocity(velocity_ramp, c_velocity_ctrl);
									actual_velocity = get_velocity(c_velocity_ctrl);

									t when timerafter(time + MSEC_STD) :> time;

								}
								end_state = 1;
								shutdown_velocity_ctrl(c_velocity_ctrl);
								printintln(position);
								if(home_state == 1)
								{
									{current_position, direction} = get_qei_position_absolute(c_qei_p4);
									printintln(current_position);
									offset = current_position - position;
									printintln(offset);
									reset_qei_count(c_qei_p4, offset);
								}

							}

							break;
					}
					if(end_state == 1)
						break;
				}
			}
		}

		on stdcore[2]:
		{

			/*Velocity Control Loop*/
			{
				ctrl_par velocity_ctrl_params;
				filter_par sensor_filter_params;
				hall_par hall_params;
				qei_par qei_params;

				init_velocity_control_param(velocity_ctrl_params);
				init_sensor_filter_param(sensor_filter_params);				init_hall_param(hall_params);
				init_qei_param(qei_params);

				velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, \
					 qei_params, SENSOR_USED, c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
			}

		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				/* PWM Loop */
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_watchdog, 	\
							c_commutation_p1, c_commutation_p2, c_commutation_p3,		\
							c_pwm_ctrl, hall_params, qei_params, commutation_params);
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, p_ifm_hall, hall_params); // channel priority 1,2..5
				}

				/* QEI Server */
				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, p_ifm_encoder, qei_params); 	 // channel priority 1,2..5
				}
			}
		}

	}

	return 0;
}
