
/**
 * \file test_qei.xc
 * \brief Test illustrates usage of qei sensor to get position and velocity information
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
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

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <qei_client.h>
#include <qei_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>
//#define ENABLE_xscope

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

void xscope_initialise_1()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 qei_position", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 qei_velocity", XSCOPE_INT,	"n");
		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}

/* Test QEI Sensor Client */
void qei_test(chanend c_qei)
{
	int position;
	int velocity;
	int direction;
	int core_id = 1;
	timer t;
	int index_count;
	int count=0;
	qei_par qei_params;
	qei_velocity_par qei_velocity_params;  			// to compute velocity from qei
	init_qei_param(qei_params);
	init_qei_velocity_params(qei_velocity_params);

#ifdef ENABLE_xscope
	xscope_initialise_1();
#endif

	while(1)
	{
		/* get position from QEI Sensor */
		{position, direction} = get_qei_position_absolute(c_qei);

		/* calculate velocity from QEI Sensor position */
		velocity = get_qei_velocity(c_qei, qei_params, qei_velocity_params);

		wait_ms(1, core_id, t);
	#ifdef ENABLE_xscope
		xscope_probe_data(0, position);
		xscope_probe_data(1, velocity);
	#else
		printstr("Position: ");
		printint(position);
		printstr(" ");
		printstr("Velocity: "); // with print velocity information will be corrupt (use xscope)
		printintln(velocity);
	#endif
	}
}

int main(void)
{
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6;				// qei channels

	par
	{
		on stdcore[1]:
		{
			/* Test QEI Sensor Client */
			qei_test(c_qei_p1);
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			/* QEI Server Loop */
			{
				qei_par qei_params;
				init_qei_param(qei_params);
				run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  		// channel priority 1,2..6
			}
		}
	}

	return 0;
}
