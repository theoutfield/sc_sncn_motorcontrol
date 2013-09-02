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
#include <drive_config.h>

#include <flash_somanet.h>
#include <internal_config.h>
#include <ctrlproto.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define HALL 1
#define QEI 2

ctrl_proto_values_t InOut;

void xscope_initialise()
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
	csp_par csp_params;

	int init = 0;
	int init_state = INIT_BUSY;

	int acc = 350;				// rpm/s
	int dec = 350;     			// rpm/s
	int velocity = 350;			// rpm
	int actual_position = 0;	// degree
	int target_position = 350;	// degree

	init_csp_param(csp_params);
	init_qei_param(qei_params);

#ifdef ENABLE_xscope_main
	xscope_initialise();
#endif

	while(1)
	{
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
		init_state = init_position_control(c_position_ctrl);
		if(init_state == INIT)
			printstrln("position control intialized");
		else
			printstrln("intialize position control failed");
	}

	if(init_state == INIT)
	{
		init_position_profile_limits(qei_params.gear_ratio, MAX_ACCELERATION, MAX_NOMINAL_SPEED);

		steps = init_position_profile(target_position, actual_position, velocity, acc, dec);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);
			position_ramp = position_profile_generate(i);
			set_position(position_ramp, c_position_ctrl);

			xscope_probe_data(1, position_ramp);
		}
		while(1)
		{
			wait_ms(1, core_id, t);
			set_position(position_ramp, c_position_ctrl);

			xscope_probe_data(1, position_ramp);
		}
	}
}


/*
int get_target_position()
{
	return InOut.in_position;
}
void send_actual_position(int actual_position)
{
	InOut.out_position = actual_position;
}
void ether_comm(chanend pdo_out, chanend pdo_in, chanend c_signal, chanend c_position_ctrl)
{
	int i = 0;
	int mode = 0;
	int core_id = 0;
	int actual_position = 0;

	timer t, t1;
	unsigned time, time1, time2;
	unsigned ts;
	int target_position;
	csp_par csp_params;
	int init = 0;

	init_csp_param(csp_params);
	init_ctrl_proto(InOut);

	//check init signal from commutation level
	init = init_commutation(c_signal);
	if(init == 1)
		printstrln("initialized commutation");
	else
		printstrln(" initialize commutation failed");

	if(init == 1)
	{
		init = 0;
		init = init_position_control(c_position_ctrl);
		if(init == 1)
			printstrln("position control intialized");
		else
			printstrln("intialize position control failed");
	}

	if(init == 1)
	{
		//test only csp
		while(1)
		{

			ctrlproto_protocol_handler_function( pdo_out, pdo_in, InOut);

			switch(InOut.ctrl_motor)
			{
				case CSP: //csp mode index

					target_position = get_target_position();
					set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);


					actual_position = get_position(c_position_ctrl);
					send_actual_position(actual_position);

					xscope_probe_data(1, target_position);
					break;
			}

			wait_ms(1, core_id, t);
		}
	}
}
*/
int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
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
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[1] :
		{
	//		ether_comm(pdo_out, pdo_in, c_signal, c_position_ctrl);   	// test CSP over ethercat with PPM on master side
		}
		on stdcore[1]:
		{
			position_profile_test(c_position_ctrl, c_signal);		  	// test PPM on slave side
			/*par
			{

				{
					{
						int init_state = INIT_BUSY;

						while(1)
						{
							printintln(init_state);
							init_state = __check_commutation_init(c_signal);
							if(init_state == INIT)
							{
								printstrln("comm intialized");
								break;
							}
						}

						init_state = INIT_BUSY;

						c_position_ctrl <: 1;
						while(1)
						{
							printintln(init_state);
							init_state = __check_position_init(c_position_ctrl);
							if(init_state == INIT)
							{
								printstrln("pos intialized");
								break;
							}
						}


					}
				}
			}*/
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
