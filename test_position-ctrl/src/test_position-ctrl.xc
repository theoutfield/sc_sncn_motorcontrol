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
#include "pwm_service_inv.h"
#include "adc_ad7949.h"
#include "test.h"
#include "pwm_config.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include "sine_table_big.h"
#include "print.h"
#include "filter_blocks.h"
#include "profile.h"
#include <position_ctrl.h>

#include <flash_Somanet.h>
#include <internal_config.h>
#include <ctrlproto.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


#define HALL 1
#define QEI 2

void position_profile_test(chanend c_position_ctrl, chanend c_signal)
{
	int samp;
	int i = 0;
	timer ts;
	unsigned time;
	int position_ramp;
	qei_par qei_params;
	csp_par csp_params;

	int acc = 350;				// rpm/s
	int dec = 350;     			// rpm/s
	int velocity = 350;			// rpm
	int actual_position = 0;	// degree
	int target_position = 350;	// degree

	init_csp(csp_params);
	init_qei(qei_params);

	//check init signal from commutation level
	while (1) {
		unsigned command, received_command = 0; //FIXME put declarations outside the loop
		select
		{
			case c_signal :> command:
				received_command = 1;
			break;
			default:
			break;
		}
		if(received_command == 1)
		{
			printstrln(" init commutation");
			break;
		}
	}

	POSITION_CTRL_ENABLE(); 	//activate position ctrl

	 // init check from position control loop
	 while(1)
	 {
		unsigned command, received_command =0; //FIXME put declarations outside the loop
		select
		{
			case POSITION_CTRL_READ(command):
				received_command = 1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
		  printstrln("pos intialised");
		  break;
		}
	 }

	init_position_profile_limits(qei_params.gear_ratio, MAX_ACCELERATION, MAX_NOMINAL_SPEED);

	samp = init_position_profile(target_position, actual_position, velocity, acc, dec);

	ts:>time;

	for(i = 1; i < samp; i++)
	{
		ts when timerafter(time+100000) :> time;
		position_ramp = position_profile_generate(i);
		xscope_probe_data(1, position_ramp);
		set_position_csp(csp_params, position_ramp, 0, 0, 0, c_position_ctrl);
	}
	while(1)
	{
		ts when timerafter(time+100000) :> time;
		xscope_probe_data(1, position_ramp);
		set_position_csp(csp_params, position_ramp, 0, 0, 0, c_position_ctrl);
	}
}



void ether_comm(chanend pdo_out, chanend pdo_in, chanend c_signal, chanend c_position_ctrl)
{
	ctrl_proto_values_t InOut;

	int i = 0;
	int mode = 0;
	int actual_position = 0;

	timer t, t1;
	unsigned time, time1, time2;
	unsigned ts;
	int target_position;
	csp_par csp_params;

	init_csp(csp_params);
	init_ctrl_proto(InOut);

	//check init signal from commutation level
	while (1) {
		unsigned command, received_command = 0; //FIXME put declarations outside the loop
		select
		{
			case c_signal :> command:
				received_command = 1;
			break;
			default:
			break;
		}
		if(received_command == 1)
		{
			printstrln(" init commutation");
			break;
		}
	}

	POSITION_CTRL_ENABLE(); 	//activate position ctrl

	 // init check from position control loop
	 while(1)
	 {
		unsigned command, received_command =0; //FIXME put declarations outside the loop
		select
		{
			case POSITION_CTRL_READ(command):
				received_command = 1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
		  printstrln("pos intialised");
		  break;
		}
	 }

	//test only csp

	t :> time;

	while(1)
	{

		ctrlproto_protocol_handler_function( pdo_out, pdo_in, InOut);

		switch(InOut.ctrl_motor)
		{

			case CSP: //csp mode index
				mode = CSP;
				target_position = InOut.in_position;
				break;

		}

		select
		{
			case t when timerafter(time + MSEC_STD) :> time:
				if(mode == CSP)
				{
					set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);
					actual_position = get_position(c_position_ctrl);
					InOut.out_position = actual_position;
					xscope_probe_data(1, target_position);
				}
				break;

		}

	}
}

int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_pwm_ctrl, c_commutation;
	chan dummy, dummy1, dummy2;
	chan signal_adc;
	chan sig_1, c_signal;
	chan c_velocity_ctrl, c_position_ctrl;

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
			ether_comm(pdo_out, pdo_in, c_signal, c_position_ctrl);  // test CSP over ethercat with PPM on master side
		}
		on stdcore[1]:
		{
			//position_profile_test(c_position_ctrl, c_signal);		  // test PPM on slave side
		}

		on stdcore[1]:
		{
			xscope_register(14, XSCOPE_CONTINUOUS, "0 actual_position", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "1 target_position", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "2 ramp", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
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

		on stdcore[2]:
		{

			{
				 ctrl_par position_ctrl_params;
				 hall_par hall_params;
				 qei_par qei_params;

				 init_position_control(position_ctrl_params);
				 init_hall(hall_params);
				 init_qei(qei_params);

				 position_control(position_ctrl_params, hall_params, qei_params, QEI, c_hall_p2,\
								  c_qei, c_position_ctrl, c_commutation);
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

				commutation_sinusoidal(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc, c_signal); // hall based sinusoidal commutation


				run_hall( p_ifm_hall, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4

				run_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}
