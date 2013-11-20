
/**
 *
 * \file test_qei.xc
 *
 * \brief Main project file
 *  Test illustrates usage of qei sensor for position and velocity information
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
#include <qei_client.h>
#include <qei_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>

//#include <flash_somanet.h>

//#define ENABLE_xscope_main

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

void xscope_initialise_1()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 hall_position", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 hall_velocity", XSCOPE_INT,	"n");
		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}

/* hall sensor test function */
void qei_test(chanend c_qei)
{
	int position;
	int velocity;
	int valid;
	int core_id = 1;
	timer t;
	qei_par qei_params;
	qei_velocity_par qei_velocity_params;  // to compute velocity from qei

	init_qei_param(qei_params);
	init_qei_velocity_params(qei_velocity_params);	// to compute velocity from qei

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1)
	{
		{position, valid} = get_qei_position(c_qei, qei_params);
		velocity = get_qei_velocity(c_qei, qei_params, qei_velocity_params);
		wait_ms(1, core_id, t);

#ifdef ENABLE_xscope_main
		xscope_probe_data(0, position);
		xscope_probe_data(1, velocity);
#else
		printstr("position");
		printint(position);
		printstr("velocity ");   // with print velocity information will be corrupt (use xscope)
		printintln(velocity);
#endif
	}
}

int main(void)
{
	chan c_adctrig, c_adc;													// adc channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5;					// qei channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl;														// pwm channels


	par
	{

		on stdcore[1]:
		{
			/* Test qei sensor */
			par
			{
				qei_test(c_qei_p1);
			}
		}



		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				/* QEI Server */
				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, p_ifm_encoder, qei_params);  		// channel priority 1,2..5
				}
			}
		}

	}

	return 0;
}
