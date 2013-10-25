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
#include "hall_server.h"
#include "hall_client.h"
#include "qei_client.h"
#include <pwm_config.h>
#include "pwm_service_inv.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include <dc_motor_config.h>
#include "profile.h"
#include "hall_qei.h"
#include <flash_somanet.h>
#include <internal_config.h>
#include "sine_table_big.h"
#include "auto_calib_qei.h"
#include <drive_config.h>
#include "adc_ad7949.h"
#include <torque_ctrl.h>
#include <test.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


int main(void) {
	chan c_adctrig, c_adc;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl, dummy, c_sync, c_sync_p1, c_calib, c_torque_ctrl;

	//etherCat Comm channels
	chan coe_in; 	///< CAN from module_ethercat to consumer
	chan coe_out; 	///< CAN from consumer to module_ethercat
	chan eoe_in; 	///< Ethernet from module_ethercat to consumer
	chan eoe_out; 	///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 	///< File from module_ethercat to consumer
	chan foe_out; 	///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	//
	par
	{
//		on stdcore[0] :
//		{
//			ecat_init();
//			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
//					foe_in, pdo_out, pdo_in);
//		}

		on stdcore[0] :
		{
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
			//xscope_initialise_1();
			par{


			{


			}
			}
		}

		on stdcore[1]:
		{
			par{
			{
				xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
						"n", XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
						XSCOPE_DISCRETE, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
						"4 ev", XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "5 pos_d",
						XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "6 vel_d", XSCOPE_INT,
						"n", XSCOPE_CONTINUOUS, "7 speed", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "8 sinepos_a", XSCOPE_UINT, "n",
						XSCOPE_CONTINUOUS, "9 sinepos_b", XSCOPE_UINT, "n",
						XSCOPE_CONTINUOUS, "10 sinepos_c", XSCOPE_UINT, "n",
						XSCOPE_CONTINUOUS, "11 sine_a", XSCOPE_UINT, "n",
						XSCOPE_CONTINUOUS, "12 sine_b", XSCOPE_UINT, "n",
						XSCOPE_CONTINUOUS, "13 sine_c", XSCOPE_UINT, "n");
				xscope_config_io(XSCOPE_IO_BASIC);
					}
			{
				qei_par qei_params;
				hall_par hall_params;
				commutation_par commutation_params;
				ctrl_par torque_ctrl_params;
				timer t;
				unsigned int time;
				int init;
				int pos_h, pos_q, dirn, f1, f2;
				init_hall_param(hall_params);
				init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
				init_torque_control_param(torque_ctrl_params);
				init_qei_param(qei_params);

				commutation_params.offset_forward = 400;
				commutation_params.offset_backward = 2730;
				commutation_sensor_select( c_commutation_p2, 2); //QEI
				set_commutation_params(c_commutation_p2, commutation_params);

			//	printstrln("calibrate start");

			//	qei_calibrate( c_signal,  c_commutation_p1, commutation_params,	hall_params, qei_params, c_hall_p4, c_qei_p4, c_calib);
//				while(1)
//				{
//					set_commt_test( c_commutation_p2) ;
//				}



		/*		t:> time;

				while(1)
				{
					t when timerafter(time + MSEC_STD) :> time;
					pos_h = get_hall_position(c_hall_p4);
					{pos_q, f1 ,f2} = get_qei_sync_position(c_qei_p4);
					xscope_probe_data(0, pos_h);
					xscope_probe_data(1, pos_q);
					xscope_probe_data(2, f2);

				}*/

				/*printintln(commutation_params.qei_forward_offset);
				printintln(commutation_params.qei_backward_offset);
				printstrln("calibrate done");*/
			}



			}
		}

		on stdcore[2]:
		{
			{

							qei_par qei_params;
							hall_par hall_params;
							commutation_par commutation_params;
							ctrl_par torque_ctrl_params;
							timer t;
							unsigned int time;
							int pos_h, pos_q, dirn, f1, f2;
							init_hall_param(hall_params);
							init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
							init_torque_control_param(torque_ctrl_params);
							init_qei_param(qei_params);
							torque_ctrl( torque_ctrl_params, hall_params, qei_params, c_adc,  c_commutation_p1,  c_hall_p4,  c_qei_p4, dummy);
				}
		/*	par
			{
				{
					qei_par qei_params;
					hall_par hall_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
					//xscope_initialise_2();

					hall_qei_sync(qei_params, hall_params, commutation_params, c_qei_p1, c_hall_p1, c_sync, c_calib);
				}
			}*/
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
					int sensor_select = 1;//hall
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p2,  c_qei_p2,\
									 c_signal, c_sync, c_commutation_p1, c_commutation_p2,\
									 c_commutation_p3, c_pwm_ctrl, sensor_select, hall_params,\
									 qei_params, commutation_params);
				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, p_ifm_hall, hall_params); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, p_ifm_encoder, qei_params);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
