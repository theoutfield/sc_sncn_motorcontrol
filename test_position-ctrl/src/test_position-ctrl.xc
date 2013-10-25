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
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "comm_loop.h"
#include "refclk.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include <dc_motor_config.h>
#include "profile.h"
#include <position_ctrl.h>
#include <drive_config.h>

#include <flash_somanet.h>
#include <internal_config.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define HALL 1
#define QEI 2


void xscope_initialise_1()
{
	xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_position", XSCOPE_INT,	"n",
						XSCOPE_CONTINUOUS, "1 target_position", XSCOPE_INT, "n");

	xscope_config_io(XSCOPE_IO_BASIC);
	return;
}

void position_profile_test(chanend c_position_ctrl, chanend c_signal)
{
	int core_id = 1;
	int i = 0;
	timer t;

	int steps;
	int position_ramp;

	qei_par qei_params;
	pp_par  pp_params;

	int init_state = INIT_BUSY;

	int acceleration 	= 350;			// rpm/s       	 variable parameters
	int deceleration 	= 350;     		// rpm/s
	int velocity 		= 350;			// rpm

	int actual_position = 0;			// degree
	int target_position = 350;			// degree

	init_qei_param(qei_params);
	init_pp_params(pp_params);

	acceleration = pp_params.base.profile_acceleration;   // or fixed parameters
	deceleration = pp_params.base.profile_deceleration;
	velocity 	 = pp_params.profile_velocity;


#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	while(1)
	{
		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
			printstrln("commutation intialized");
			break;
		}
	}
	if(init_state == INIT)
	{
		init_state = INIT_BUSY;
		init_state = init_position_control(c_position_ctrl);
		if(init_state == INIT)
			printstrln("position control intialized");
		else
			printstrln("intialize position control failed");
	}

	if(init_state == INIT)
	{
		init_position_profile_limits(qei_params.gear_ratio, MAX_ACCELERATION, MAX_PROFILE_VELOCITY);

		actual_position = get_position(c_position_ctrl); //degree * 10000
		steps = init_position_profile(target_position*10000, actual_position, velocity, acceleration, deceleration);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);
			position_ramp = position_profile_generate(i);
			set_position(position_ramp, c_position_ctrl);
			actual_position = get_position(c_position_ctrl);

			xscope_probe_data(0, actual_position);
			xscope_probe_data(1, position_ramp);
		}

		wait_ms(100, core_id, t);
		actual_position = get_position(c_position_ctrl); //degree * 10000
		target_position = 1;							 //degree
		steps = init_position_profile(target_position*10000, actual_position, velocity, acceleration, deceleration);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);
			position_ramp = position_profile_generate(i);
			set_position(position_ramp, c_position_ctrl);
			actual_position = get_position(c_position_ctrl);

			xscope_probe_data(0, actual_position);
			xscope_probe_data(1, position_ramp);
		}

		while(1)
		{
			wait_ms(1, core_id, t);
			set_position(position_ramp, c_position_ctrl);
			actual_position = get_position(c_position_ctrl);

			xscope_probe_data(0, actual_position);
			xscope_probe_data(1, position_ramp);
		}
	}
}

int main(void)
{
	chan c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_position_ctrl;

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
		on stdcore[0] :
		{
			ecat_init();

			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[1]:
		{
			position_profile_test(c_position_ctrl, c_signal);		  	// test PPM on slave side
		}


		on stdcore[2]:
		{
			{
				 ctrl_par position_ctrl_params;
				 hall_par hall_params;
				 qei_par qei_params;

				 init_position_control_param(position_ctrl_params);
				 init_hall_param(hall_params);
				 init_qei_param(qei_params);

				 position_control(position_ctrl_params, hall_params, qei_params, QEI, c_hall_p2,\
						 c_qei_p1, c_position_ctrl, c_commutation_p3);
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

				{
					int sensor_select = 1;
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2,
								 c_signal, c_sync, c_commutation_p1, c_commutation_p2,
								 c_commutation_p3, c_pwm_ctrl, sensor_select, hall_params,
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
