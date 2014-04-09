
/**
 * \file test_hall.xc
 * \brief Test illustrates usage of hall sensor to get position and velocity information
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 *  All rights reserved.
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
#include <hall_client.h>
#include <hall_server.h>
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
		xscope_register(2, XSCOPE_CONTINUOUS, "0 hall_position", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 hall_velocity", XSCOPE_INT,	"n");
		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}


/* Test Hall Sensor Client */
void hall_test(chanend c_hall)
{
	int position;
	int velocity;
	int core_id = 1;
	timer t;
	int direction;
	hall_par hall_params;
	init_hall_param(hall_params);

#ifdef ENABLE_xscope
	xscope_initialise_1();
#endif

	while(1)
	{
		/* get position from Hall Sensor */
		{position, direction} = get_hall_position_absolute(c_hall);

		/* get velocity from Hall Sensor */
		velocity = get_hall_velocity(c_hall, hall_params);

#ifdef ENABLE_xscope
		xscope_probe_data(0, position);
		xscope_probe_data(1, velocity);
#else
		printstr("Position: ");
		printint(position);
		printstr(" ");
		printstr("Velocity: ");
		printintln(velocity);
#endif

	}
}

int main(void)
{
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels

	par
	{
		on stdcore[1]:
		{
			/* Test Hall Sensor Client */
			par
			{
				hall_test(c_hall_p1);
			}
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
				}
			}
		}

	}

	return 0;
}
