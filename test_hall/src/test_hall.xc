/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <ioports.h>
#include <hall_client.h>
#include <hall_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>
#include <drive_config.h>
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
void hall_test(chanend c_hall)
{
	int position;
	int velocity;
	int core_id = 1;
	timer t;
	hall_par hall_params;
	init_hall_param(hall_params);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1)
	{
		position = get_hall_position(c_hall);
		velocity = get_hall_velocity(c_hall, hall_params);
		wait_ms(1, core_id, t);

#ifdef ENABLE_xscope_main
		xscope_probe_data(0, position);
		xscope_probe_data(1, velocity);
#else
		printstr("position");
		printint(position);
		printstr("velocity ");
		printintln(velocity);
#endif
	}
}

int main(void)
{
	chan c_adctrig, c_adc;													// adc channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;				// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl;														// pwm channels
	chan c_qei;																// qei channels


	par
	{

		on stdcore[1]:
		{
			/* Test hall sensor */
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
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, p_ifm_hall, hall_params); // channel priority 1,2..4
				}
			}
		}

	}

	return 0;
}
