
/**
 *
 * \file test_torque-ctrl.xc
 * \brief Test illustrates usage of profile torque control
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date
 *
 */
/*
 * Copyright (c) 2013, Synapticon GmbH
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

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <hall_server.h>
#include <qei_server.h>
#include <pwm_service_inv.h>
#include <adc_server_ad7949.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>
#include <torque_ctrl_server.h>
#include <profile_control.h>
#include <internal_config.h>
#include <drive_config.h>
#include "torque_ctrl_client.h"
#include <profile.h>
#include <test.h>

//#define ENABLE_xscope

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

void xscope_initialise_1()
{
	xscope_register(2, XSCOPE_CONTINUOUS, "0 target_torque", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "1 actual_torque", XSCOPE_INT, "n");
	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}

/* Test Profile Torque Function */
void profile_torque_test(chanend c_torque_ctrl)
{
	int target_torque = 200; 	//(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 100;  	//(desired torque_slope/torque_constant)  * IFM resolution
	cst_par cst_params; int actual_torque; timer t; unsigned int time;
	init_cst_param(cst_params);

#ifdef ENABLE_xscope
	xscope_initialise_1();
#endif

	/* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	target_torque = 0;
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	target_torque = -200;
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);
	while(1)
	{
		actual_torque = get_torque(c_torque_ctrl)*cst_params.polarity;
		t when timerafter(time + MSEC_STD) :> time;
#ifdef ENABLE_xscope
		xscope_probe_data(0, actual_torque);
#endif
	}
}

int main(void)
{

	// Motor control channels
	chan c_adc, c_adctrig;													// adc channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6 ; 		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl;														// pwm channel
	chan c_torque_ctrl,c_velocity_ctrl, c_position_ctrl;					// torque control channel
	chan c_watchdog; 														// watchdog channel

	par
	{

		/* Test Profile Torque Function */
		on stdcore[1]:
		{
			profile_torque_test(c_torque_ctrl);
			//torque_ctrl_unit_test(c_torque_ctrl, c_qei_p4, c_hall_p4);
		}

		on stdcore[2]:
		{
			par
			{
				/* Torque Control Loop */
				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;
					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_torque_control_param(torque_ctrl_params);
					torque_control( torque_ctrl_params, hall_params, qei_params, SENSOR_USED,
							c_adc, c_commutation_p1,  c_hall_p3,  c_qei_p3, c_torque_ctrl);
				}
			}
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				/* ADC Loop */
				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,\
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,\
						p_ifm_adc_misob);

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
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, 	\
							c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl,\
							p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2,\
							hall_params, qei_params, commutation_params);
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..4
				}

				/* QEI Server */
				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  // channel priority 1,2..4
				}
			}
		}

	}

	return 0;
}
