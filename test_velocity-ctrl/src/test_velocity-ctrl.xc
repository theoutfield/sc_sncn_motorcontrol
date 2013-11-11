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
#include "ioports.h"
#include "hall_server.h"
#include "hall_client.h"
#include "qei_server.h"
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl_server.h"
#include <xscope.h>
#include "profile.h"
#include <flash_somanet.h>
#include <internal_config.h>
#include <dc_motor_config.h>
#include <drive_config.h>
#include <profile_control.h>

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


//test PVM
void profile_velocity_test(chanend c_velocity_ctrl)
{
	int target_velocity = 2000;	 		// rpm
	int acceleration 	= 2000;			// rpm/s
	int deceleration 	= 1050;			// rpm/s

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);

	target_velocity = 0;
	set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, c_velocity_ctrl);
}

int main(void) {
	chan c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_velocity_ctrl;

	//etherCat Comm channels
	chan coe_in; 		///< CAN from module_ethercat to consumer
	chan coe_out; 		///< CAN from consumer to module_ethercat
	chan eoe_in; 		///< Ethernet from module_ethercat to consumer
	chan eoe_out; 		///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 		///< File from module_ethercat to consumer
	chan foe_out; 		///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	//
	par
	{
		on stdcore[0] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,\
					foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			firmware_update(foe_out, foe_in, c_sig_1); 		// firmware update over EtherCat
		}

		on stdcore[1]:
		{
			profile_velocity_test(c_velocity_ctrl);			// test PVM on node
		}

		on stdcore[2]:
		{
			par
			{
				{
					ctrl_par velocity_ctrl_params;
					filter_par sensor_filter_params;
					hall_par hall_params;
					qei_par qei_params;

					init_velocity_control_param(velocity_ctrl_params);
					init_sensor_filter_param(sensor_filter_params);
					init_hall_param(hall_params);
					init_qei_param(qei_params);

					velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, \
						 qei_params, QEI, c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
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
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_sync,\
							c_commutation_p1, c_commutation_p2, c_commutation_p3,\
							c_pwm_ctrl, hall_params, qei_params, commutation_params);
				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, p_ifm_hall, hall_params); // channel priority 1,2..5
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, p_ifm_encoder, qei_params);  // channel priority 1,2..5
				}
			}
		}

	}

	return 0;
}
