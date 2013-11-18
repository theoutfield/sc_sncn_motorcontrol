/*
 *
 * \file test_adc_ext_potentiometer.xc
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
#include <adc_server_ad7949.h>
#include <adc_client_ad7949.h>
#include <hall_server.h>
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <xscope.h>
#include <bldc_motor_config.h>
#include <drive_config.h>
//#include <flash_somanet.h>

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

/* External potentiometer test function */
void external_pot_test(chanend c_adc)
{
	int external_pot1;
	int external_pot2;
	int core_id = 1;
	timer t;

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1)
	{
		{external_pot1 , external_pot2} = get_adc_external_potentiometer_ad7949(c_adc);
		wait_ms(1, core_id, t);

#ifdef ENABLE_xscope_main
		xscope_probe_data(0, external_pot1);
		xscope_probe_data(1, external_pot2);
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
			/* Test external potentiometers */
			par
			{
				external_pot_test(c_adc);
			}
		}



		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				/* ADC loop (only if motor control is not used) */
			//	adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob );

				/* ADC triggered loop (only if motor control is used) */
				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
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
					commutation_sinusoidal(c_hall_p1,  c_qei,\
							 c_signal, c_commutation_p1, c_commutation_p2,\
							 c_commutation_p3, c_pwm_ctrl, hall_params,\
							 qei_params, commutation_params);
				}

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
