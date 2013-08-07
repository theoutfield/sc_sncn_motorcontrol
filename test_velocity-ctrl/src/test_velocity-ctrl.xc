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
#include <flash_somanet.h>
#include <internal_config.h>
#include <ctrlproto.h>

#include <drive_config.h>
#include "profile_test.h"

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

ctrl_proto_values_t InOut;
/*
int get_target_velocity()
{
	return InOut.in_speed;
}
void send_actual_velocity(int actual_velocity)
{
	InOut.out_speed = actual_velocity;
}


void ether_comm(chanend pdo_out, chanend pdo_in, chanend c_signal, chanend c_velocity_ctrl)
{
	int i = 0;
	int mode = 0;
	int core_id = 0;

	int target_velocity;
	int actual_velocity = 0;

	timer t;

	int init = 0;

	csv_par csv_params;


	init_csv_param(csv_params);

	init_ctrl_proto(InOut);

	//check init signal from commutation loop
	init = init_commutation(c_signal);
	if(init == 1)
		printstrln("initialized commutation");
	else
		printstrln(" initialize commutation failed");


	if(init == 1)
	{
		init = 0;
		init = init_velocity_control(c_velocity_ctrl);
		if(init == 1)
			printstrln("velocity control intialized");
		else
			printstrln("intialize velocity control failed");
	}

	if(init == 1)
	{
		//test only csv
		while(1)
		{
			ctrlproto_protocol_handler_function( pdo_out, pdo_in, InOut);

			switch(InOut.ctrl_motor)
			{
				case CSV: 	//csv mode index

					target_velocity = get_target_velocity();
					set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);

					actual_velocity = get_velocity(c_velocity_ctrl);
					send_actual_velocity(actual_velocity);

					xscope_probe_data(0, actual_velocity);
					xscope_probe_data(1, target_velocity);
					break;
			}

			wait_ms(1, core_id, t);
		}
	}
}
*/
void xscope_initialise()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}
//test PVM
void profile_velocity_test(chanend c_signal, chanend c_velocity_ctrl)
{
	int i;
	int core_id = 1;

	int steps;
	int velocity_ramp;

	int actual_velocity = 0;     // rpm
	int target_velocity = 2000;	 // rpm
	int acc = 8000;				 // rpm/s
	int dec = 1050;				 // rpm/s

	timer t;
	unsigned time;

	csv_par csv_params;

	int init = 0;
	int init_state = INIT_BUSY;

	init_csv_param(csv_params);

#ifdef ENABLE_xscope_main
	xscope_initialise();
#endif
	while(1)
	{
		//printintln(init_state);
		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
			printstrln("comm intialized");
			break;
		}
	}
	if(init_state == INIT)
	{
		init_state = INIT_BUSY;
		init_state = init_velocity_control(c_velocity_ctrl);
		if(init_state == INIT)
			printstrln("velocity control intialized");
		else
			printstrln("intialize velocity control failed");
	}

	if(init_state == INIT)
	{

		steps = init_velocity_profile(target_velocity, actual_velocity * csv_params.polarity, acc, dec);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);

			velocity_ramp = velocity_profile_generate(i);
			set_velocity_csv(csv_params, velocity_ramp, 0, 0, c_velocity_ctrl);

			actual_velocity = get_velocity(c_velocity_ctrl);

			xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1, velocity_ramp);
		}

		wait_s(2, core_id, t);  // wait 2 seconds

		actual_velocity = get_velocity(c_velocity_ctrl);	// rpm
		target_velocity = 0; 								// rpm
		acc = 8000; 										// rpm/s
		dec = 1050;											// rpm/s

		steps = init_velocity_profile(target_velocity, actual_velocity * csv_params.polarity, acc, dec);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);

			velocity_ramp = velocity_profile_generate(i);
			set_velocity_csv(csv_params, velocity_ramp, 0, 0, c_velocity_ctrl);

			actual_velocity = get_velocity(c_velocity_ctrl);

			xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1, velocity_ramp);

		}

		while(1)
		{
			wait_ms(1, core_id, t);

			set_velocity_csv(csv_params, velocity_ramp, 0, 0, c_velocity_ctrl);

			actual_velocity = get_velocity(c_velocity_ctrl);

			xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1, velocity_ramp);
		}
	}
}

int main(void) {
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl;

	//etherCat Comm channels
	chan coe_in; ///< CAN from module_ethercat to consumer
	chan coe_out; ///< CAN from consumer to module_ethercat
	chan eoe_in; ///< Ethernet from module_ethercat to consumer
	chan eoe_out; ///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; ///< File from module_ethercat to consumer
	chan foe_out; ///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	//
	par
	{
		on stdcore[0] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
					foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[1] :
		{
			//ether_comm(pdo_out, pdo_in, c_signal, c_velocity_ctrl);   // test CSV over ethercat with PVM on master side
		}
		on stdcore[1]:
		{
			profile_velocity_test(c_signal, c_velocity_ctrl);			// test PVM on slave side

			/*par
			{

				{
					{
						int init_state = INIT_BUSY;

						while(1)
						{
							//printintln(init_state);
							init_state = __check_commutation_init(c_signal);
							if(init_state == INIT)
							{
								printstrln("comm intialized");
								break;
							}
						}

						init_state = INIT_BUSY;

						c_velocity_ctrl <: 1;
						while(1)
						{
							printintln(init_state);
							init_state = __check_velocity_init(c_velocity_ctrl);
							if(init_state == INIT)
							{
								printstrln("vel intialized");
								break;
							}
						}


					}
				}
			}*/
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
							 qei_params, 2, c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
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
					init_hall_param(hall_params);
					commutation_sinusoidal(hall_params, c_hall_p1, c_pwm_ctrl, c_signal_adc, c_signal,
							c_commutation_p1, c_commutation_p2, c_commutation_p3);					 // hall based sinusoidal commutation
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
