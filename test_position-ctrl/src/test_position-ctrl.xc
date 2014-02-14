
/**
 *
 * \file test_position-ctrl.xc
 *
 * \brief Main project file
 *  Test illustrates usage of profile position control
 *
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
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <xscope.h>
#include <qei_server.h>
#include <bldc_motor_config.h>
#include <profile.h>
#include <position_ctrl_server.h>
#include <drive_config.h>
#include <profile_control.h>
#include <internal_config.h>
#include <flash_somanet.h>
#include "position_ctrl_client.h"
//#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


void xscope_initialise_1()
{
	xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_position", XSCOPE_INT,	"n",
						XSCOPE_CONTINUOUS, "1 target_position", XSCOPE_INT, "n");

	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}


/* Test Profile Position function */
void position_profile_test(chanend c_position_ctrl, chanend c_qei)
{
	int init_state;
	int target_position = 350;			// deg
	int velocity 		= 350;			// rpm
	int acceleration 	= 350;			// rpm/s
	int deceleration 	= 350;     		// rpm/s
	int turns = 1;
	init_position_profile_limits(GEAR_RATIO, MAX_ACCELERATION, MAX_PROFILE_VELOCITY);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif


	//set_qei_turns(c_qei, turns);
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
	//set_qei_turns(c_qei, turns);
	/*reset_qei_count(c_qei, 0);
	shutdown_position_ctrl(c_position_ctrl);
*/
	target_position = 350; 	//degree
	set_profile_position(target_position, velocity, acceleration, deceleration, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT, c_position_ctrl);
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6;		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl, c_adctrig;												// pwm channels
	chan c_position_ctrl;													// position control channel
	chan c_watchdog; 														// watchdog channel

	// EtherCat Communication channels
	chan coe_in; 		//< CAN from module_ethercat to consumer
	chan coe_out; 		//< CAN from consumer to module_ethercat
	chan eoe_in; 		//< Ethernet from module_ethercat to consumer
	chan eoe_out; 		//< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 		//< File from module_ethercat to consumer
	chan foe_out; 		//< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;
	chan c_sig_1;

	par
	{
		/* Ethercat Communication Handler Loop */
		on stdcore[0] :
		{
			ecat_init();

			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
		}

		/* Firmware Update Loop */
		on stdcore[0] :
		{
			firmware_update(foe_out, foe_in, c_sig_1); 		// firmware update over EtherCat
		}

		/* Test Profile Position function*/
		on stdcore[1]:
		{
			position_profile_test(c_position_ctrl, c_qei_p5);		  	// test PPM on slave side
		}


		on stdcore[2]:
		{
			/* Position Control Loop */
			{
				 ctrl_par position_ctrl_params;
				 hall_par hall_params;
				 qei_par qei_params;

				 init_position_control_param(position_ctrl_params);
				 init_hall_param(hall_params);
				 init_qei_param(qei_params);

				 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2,\
						 c_qei_p1, c_position_ctrl, c_commutation_p3);
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
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 			// initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_watchdog, \
							c_commutation_p1, c_commutation_p2, c_commutation_p3, \
							c_pwm_ctrl, hall_params, qei_params, commutation_params);
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..5
				}

				/* QEI Server */
				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  	// channel priority 1,2..5
				}

			}
		}

	}

	return 0;
}
