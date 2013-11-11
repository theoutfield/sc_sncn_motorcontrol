/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *\Motor 3 motion profile size optimized code for position ctrl loops
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include "ioports.h"
#include "adc_server_ad7949.h"
#include "adc_client_ad7949.h"
#include "hall_server.h"
#include "hall_client.h"
#include "pwm_service_inv.h"
#include "comm_loop.h"
#include "refclk.h"
#include <xscope.h>
#include <dc_motor_config.h>
#include <flash_somanet.h>
#include <drive_config.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

void xscope_initialise_1()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 external_pot", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 external_pot1", XSCOPE_INT,	"n");
		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}

int main(void) {
	chan c_adctrig;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_adc, c_qei;


	par
	{

		on stdcore[1]:
		{
			par
			{
				{
					int external_pot1, external_pot2;
					unsigned int time;
					timer t;
					t :> time;
					xscope_initialise_1();
					while(1)
					{
						t when timerafter(time + 500*USEC_STD) :> time;
						{external_pot1 , external_pot2} = get_adc_external_potentiometer_ad7949(c_adc);
						xscope_probe_data(0, external_pot1);
						xscope_probe_data(1, external_pot2);
					}
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
			//	adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob );

				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					int sensor_select = 1;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei,\
							 c_signal, c_sync, c_commutation_p1, c_commutation_p2,\
							 c_commutation_p3, c_pwm_ctrl, hall_params,\
							 qei_params, commutation_params);
				}

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
