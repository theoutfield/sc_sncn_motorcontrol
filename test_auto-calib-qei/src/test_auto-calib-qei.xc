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
#include "hall-qei.h"
#include <flash_somanet.h>
#include <internal_config.h>
#include "sine_table_big.h"

#include <drive_config.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;
static t_pwm_control pwm_ctrl1;
void xscope_initialise_1()
{
	{
		xscope_register(3, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n",
							XSCOPE_CONTINUOUS, "2 target_vel2", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}
//test PVM
void profile_velocity_test(chanend c_signal, chanend c_velocity_ctrl,  chanend c_hall, chanend c_qei)
{
	int i;
	int core_id = 1;

	int steps;
	int velocity_ramp;

	int actual_velocity = 0;     		// rpm
	int target_velocity =7000;	 		// rpm
	int acceleration = 2000;			// rpm/s      variable parameters
	int deceleration = 1050;			// rpm/s

	timer t;
	unsigned int time;

	int angle_clk = 45;
	int min_angle = 1*4095/360;
	int max_velocity_r = 0;

	pv_par pv_params;
	commutation_par commutation_params;
	hall_par hall_params;
	qei_par qei_params;


	int init = 0;
	int init_state = INIT_BUSY;

	init_pv_params(pv_params);
	init_commutation_param(commutation_params);
	init_hall_param(hall_params);
	init_qei_param(qei_params);

	acceleration = pv_params.profile_acceleration;   // or fixed parameters
	deceleration = pv_params.profile_deceleration;

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif
	while(1)
	{
		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
		//	printstrln("commutation intialized");
			break;
		}
	}
	if(init_state == INIT)
	{
		init_state = INIT_BUSY;
		init_state = init_velocity_control(c_velocity_ctrl);
		//if(init_state == INIT)	printstrln("velocity control intialized");
		//else			printstrln("intialize velocity control failed");
	}

	if(init_state == INIT)
	{
		steps = init_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration, 8000);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);
			velocity_ramp = velocity_profile_generate(i);
			set_velocity(velocity_ramp, c_velocity_ctrl);
			actual_velocity = get_velocity(c_velocity_ctrl);
//
			xscope_probe_data(0, actual_velocity);
		//	xscope_probe_data(0,  get_qei_velocity( c_qei, qei_params));
		//	xscope_probe_data(0,  _get_qei_velocity_pwm_resolution( c_qei, qei_params));
			//xscope_probe_data(1, _get_hall_velocity_pwm_resolution(c_hall, hall_params));
		}
t:>time;
		while(1)
		{
		//	wait_ms(1, core_id, t);

			t when timerafter(time+5555) :> time;
			//commutation_params.angle_offset_clkwise = 600;
//			angle_clk = angle_clk + 1;
//			if(angle_clk > 1251)
//				angle_clk = 1251;
//			commutation_params.angle_offset_cclkwise = angle_clk;
//			set_commutation_params( c_commutation, commutation_params);
//			wait_ms(50, core_id, t);
			actual_velocity = get_velocity(c_velocity_ctrl);

//			forw_calib(commutation_params, angle_clk);
//			commutation_params.angle_offset_clkwise = angle_clk;
//			set_commutation_params(c_commutation, commutation_params);
//			wait_ms(150, 1, t);
			//actual_velocity = get_velocity(c_velocity_ctrl);
		//	printintln(actual_velocity);
			xscope_probe_data(0, actual_velocity);
		//	xscope_probe_data(0,  get_qei_velocity( c_qei, qei_params));
			//xscope_probe_data(0,  _get_qei_velocity_pwm_resolution( c_qei, qei_params));
		//	xscope_probe_data(, _get_hall_velocity_pwm_resolution(c_hall, hall_params));
		//	xscope_probe_data(1, angle_clk);
//			if(actual_velocity > max_velocity_r)
//				max_velocity_r = actual_velocity;
		//	printintln(max_velocity_r);


		}
	}
}

int main(void) {
	chan c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl, dummy, c_sync;

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
		}

		on stdcore[1]:
		{
			profile_velocity_test(c_signal, c_velocity_ctrl,  c_hall_p4, c_qei_p4);			// test PVM on slave side


//			{
//				int i;
//				for(i=0;i<1024;i++)
//				{
//					printintln( sine_reduce(i));
//				}
//
//			}
		}

		on stdcore[2]:
		{
			par
			{

				{
					 ctrl_par velocity_ctrl_params;
					 filt_par sensor_filter_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_velocity_control_param(velocity_ctrl_params);
					 init_sensor_filter_param(sensor_filter_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,\
							 qei_params, 2, c_hall_p3, c_qei_p3, c_velocity_ctrl, c_commutation_p2);

				}

				{
					qei_par qei_params;
					hall_par hall_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					hall_qei_sync(qei_params, hall_params, c_qei_p1, c_hall_p2, c_sync);

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

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
//				{
//					hall_par hall_params;
//					commutation_par commutation_params;
//					init_hall_param(hall_params);
//					init_commutation_param(commutation_params); // initialize commutation params
//					test_delay_comm( hall_params, commutation_params, c_hall_p1, c_pwm_ctrl,
//							c_signal, c_commutation_p1, c_commutation_p2, c_commutation_p3);
//				}

				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					int sensor_select = 2;//hall
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params); // initialize commutation params
					commutation_sinusoidal(sensor_select, hall_params, qei_params, commutation_params, c_hall_p1, c_qei_p2, c_pwm_ctrl, c_signal_adc, c_signal,
							 c_sync, c_commutation_p1, c_commutation_p2, c_commutation_p3);					 // hall based sinusoidal commutation
				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(p_ifm_hall, hall_params, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(p_ifm_encoder, qei_params, c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
