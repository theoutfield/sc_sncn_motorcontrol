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
#include "comm_loop_server.h"
#include "refclk.h"
#include "velocity_ctrl_server.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include <bldc_motor_config.h>
#include "profile.h"
#include "hall_qei.h"
#include <flash_somanet.h>
#include <internal_config.h>
#include "sine_table_big.h"
#include "auto_calib_qei.h"
#include <drive_config.h>
#include "adc_server_ad7949.h"
#include <torque_ctrl_server.h>
#include <torque_ctrl_client.h>
#include <velocity_ctrl_server.h>
#include <profile_control.h>
#include <test.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


void xscope_initialise_1()
{
	xscope_register(2, XSCOPE_CONTINUOUS, "0 position", XSCOPE_INT, "n",
						XSCOPE_CONTINUOUS, "1 actual_torque", XSCOPE_INT, "n");
	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}

/* Test Profile Torque Function */
void profile_torque_test(chanend c_torque_ctrl)
{
	int target_torque = -100;  //(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 50;  //(desired torque_slope/torque_constant)  * IFM resolution
	cst_par cst_params;
	init_cst_param(cst_params);



	set_profile_torque( target_torque, torque_slope, cst_params, c_torque_ctrl);


//	set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
//	shutdown_torque_ctrl(c_torque_ctrl);
}

/**
 *  \return 0 init failed 1 init success
 */
int hall_sensor_initialise(chanend c_torque_ctrl, chanend c_hall, cst_par &cst_params)
{
	int i;
	int steps;
	int target_torque = cst_params.max_torque/5; 	// 20% max torque
	int torque_slope = (target_torque * 2)/3; 		// 1.5 sec
	int torque_ramp;
	int direction;
	int position;
	int actual_torque;
	timer t;
	unsigned int time;
	int init = INIT_BUSY;
	int time_wait = 0;
	int flag = 0;

	int init_state = __check_torque_init(c_torque_ctrl);
	if(init_state == INIT_BUSY)
	{
		init_state = init_torque_control(c_torque_ctrl);
		if(init_state == INIT)
		{
			printstrln("torque control intialized");
		}
	}

	actual_torque = get_torque(cst_params , c_torque_ctrl)*cst_params.polarity;
	steps = init_linear_profile(target_torque, actual_torque, torque_slope, torque_slope, cst_params.max_torque);
	t:>time;
	for(i = 1; i<steps; i++)
	{
		torque_ramp =  linear_profile_generate(i);
		set_torque( torque_ramp, cst_params , c_torque_ctrl);
		actual_torque = get_torque(cst_params , c_torque_ctrl)*cst_params.polarity;
		{position, direction} = get_hall_position_absolute(c_hall);
		flag = 0;
		if(position >= 4096 || position <= -4096) //init done
		{
			printstrln("init hall done");
			set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
			flag = 1;
			break;
			//set_torque( torque_ramp, cst_params , c_torque_ctrl);
		}
		else if(time_wait == 1500) //1.5 sec
		{
			printstrln("motor stall no over current");
			set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
		}
			//change direction
		else if (actual_torque > cst_params.max_torque)
		{
			printstrln("motor stall");
			set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
		}
			// shutdown n set flag to start again in opposite direction & reset time_wait

		time_wait++;
		t when timerafter(time + MSEC_STD) :> time;
		xscope_probe_data(1, torque_ramp);
		xscope_probe_data(0, actual_torque);
	}
	while(!flag)
	{
		actual_torque = get_torque(cst_params , c_torque_ctrl)*cst_params.polarity;
		{position, direction} = get_hall_position_absolute(c_hall);
		if(position >= 4096 || position <= -4096) //init done
		{
			printstrln("init hall done");
			set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
			break;
		}
		else if(time_wait == 1500) //1.5 sec
		{
			printstrln("motor stall no over current");
		}
			//change direction
		else if (actual_torque > cst_params.max_torque)
		{
			printstrln("motor stall");
			// shutdown n set flag to start again in opposite direction & reset time_wait
		}

		time_wait++;
	}
}
//enable_motor_test(c_commutation);
int main(void)
{
	chan c_adctrig, c_adc;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl, dummy, c_sync, c_sync_p1, c_calib, c_torque_ctrl, c_watchdog;

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
		on stdcore[1]:
		{


			{
				cst_par cst_params;
				init_cst_param(cst_params);
				hall_sensor_initialise(c_torque_ctrl, c_hall_p4, cst_params);
			}
			// set_torque_test(c_torque_ctrl, c_velocity_ctrl); //local ctrl test routine

			//profile_torque_test(c_torque_ctrl);
//			{
//				qei_par qei_params;
//				hall_par hall_params;
//				commutation_par commutation_params;
//				ctrl_par torque_ctrl_params;
//				timer t;
//				unsigned int time;
//				int init;
//				int pos_h, pos_q, dirn, f1, f2;
//
//				{
//					xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
//							"n", XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT, "n",
//							XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
//							XSCOPE_DISCRETE, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
//							"4 ev", XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "5 pos_d",
//							XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "6 vel_d", XSCOPE_INT,
//							"n", XSCOPE_CONTINUOUS, "7 speed", XSCOPE_INT, "n",
//							XSCOPE_CONTINUOUS, "8 sinepos_a", XSCOPE_UINT, "n",
//							XSCOPE_CONTINUOUS, "9 sinepos_b", XSCOPE_UINT, "n",
//							XSCOPE_CONTINUOUS, "10 sinepos_c", XSCOPE_UINT, "n",
//							XSCOPE_CONTINUOUS, "11 sine_a", XSCOPE_UINT, "n",
//							XSCOPE_CONTINUOUS, "12 sine_b", XSCOPE_UINT, "n",
//							XSCOPE_CONTINUOUS, "13 sine_c", XSCOPE_UINT, "n");
//					xscope_config_io(XSCOPE_IO_BASIC);
//				}
//				init_hall_param(hall_params);
//				init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
//				init_torque_control_param(torque_ctrl_params);
//				init_qei_param(qei_params);
//
//	//			commutation_params.offset_forward = 400;
//	//			commutation_params.offset_backward = 2730;
//	//			commutation_sensor_select( c_commutation_p2, 2); //QEI
//	//			set_commutation_params(c_commutation_p2, commutation_params);
//
//				printstrln("calibrate start");
//				//set_commutation_sinusoidal(c_commutation_p1, 500);
//				qei_calibrate( c_commutation_p2, commutation_params, hall_params, qei_params, c_hall_p4, c_qei_p4, c_calib);
////				while(1)
////				{
////					set_commt_test( c_commutation_p2) ;
////				}
//				set_qei_sync_offset(c_qei_p4, commutation_params.qei_forward_offset, commutation_params.qei_backward_offset);
//
//				set_commutation_sinusoidal(c_commutation_p2, -400);
//				t:> time;
//
//				while(1)
//				{
//					t when timerafter(time + MSEC_STD) :> time;
//					pos_h = get_hall_position(c_hall_p4);
//					{pos_q, f1 ,f2} = get_qei_sync_position(c_qei_p4);
//					//{pos_q, f1} = get_qei_position(c_qei_p4, qei_params);
//					xscope_probe_data(0, pos_h);
//					xscope_probe_data(1, pos_q);
//					xscope_probe_data(2, f2);
//					xscope_probe_data(3, f1);
//				}
//
//			//	commutation_sensor_select( c_commutation_p2, 2); //QEI
//			//	commutation_params.offset_forward = 682;
//			//	commutation_params.offset_backward = 2730;
//				//set_commutation_params(c_commutation_p2, commutation_params);
//				//set_commutation_sinusoidal(c_commutation_p2, 1400);
//
//			}




		}

		on stdcore[2]:
		{
			par
			{
				/* Torque Control Loop */
				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;
					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_torque_control_param(torque_ctrl_params);
					xscope_initialise_1();
					torque_control( torque_ctrl_params, hall_params, qei_params, SENSOR_USED,
							c_adc, c_commutation_p1,  c_hall_p2,  c_qei_p2, c_torque_ctrl);
				}


				/*Velocity Control Loop*/
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
						 qei_params, SENSOR_USED, c_hall_p3, c_qei_p3, c_velocity_ctrl, c_commutation_p2);
				}

			}//par
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
					commutation_sinusoidal(c_hall_p1,  c_qei_p1,\
									 c_signal, c_watchdog, c_commutation_p1, c_commutation_p2,\
									 c_commutation_p3, c_pwm_ctrl, hall_params,\
									 qei_params, commutation_params);
				}

				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
