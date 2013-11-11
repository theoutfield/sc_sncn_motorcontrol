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
#include "adc_server_ad7949.h"
#include "test.h"
#include "comm_loop.h"
#include "refclk.h"
#include <xscope.h>
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include <torque_ctrl_server.h>
#include <profile_control.h>
#include <flash_somanet.h>
#include <internal_config.h>

#define ENABLE_xscope_main

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

//test PTM
void profile_torque_test(chanend c_torque_ctrl)
{
	int target_torque = 6600;  //mNm  * current sensor resolution
	int torque_slope  = 6600;  //torque slope
	cst_par cst_params;
	init_cst_param(cst_params);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	target_torque = 0;
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);

	target_torque = -6000;
	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);
}

int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan sync_output;
	chan c_pwm_ctrl;
	chan dummy, dummy1, dummy2;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_torque_ctrl, signal_ctrl, c_calib, c_req, c_vel;

	//etherCat Comm channels
	chan coe_in; 			///< CAN from module_ethercat to consumer
	chan coe_out; 			///< CAN from consumer to module_ethercat
	chan eoe_in; 			///< Ethernet from module_ethercat to consumer
	chan eoe_out; 			///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 			///< File from module_ethercat to consumer
	chan foe_out; 			///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	chan c_test_in;
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
			profile_torque_test(c_torque_ctrl);
		}

		on stdcore[2]:
		{
			par
			{

				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;
					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_torque_control_param(torque_ctrl_params);
					torque_control( torque_ctrl_params, hall_params, qei_params, c_adc, \
							c_commutation_p1,  c_hall_p3,  c_qei_p3, c_torque_ctrl);
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

				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal, c_sync, \
							c_commutation_p1, c_commutation_p2, c_commutation_p3, \
							c_pwm_ctrl, hall_params, qei_params, commutation_params);
				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, p_ifm_hall, hall_params); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, p_ifm_encoder, qei_params);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
