/*
 *
 * \file test_qei.xc
 * \brief Main project file
 *
 *
 *
 *  \author Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
 *
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
