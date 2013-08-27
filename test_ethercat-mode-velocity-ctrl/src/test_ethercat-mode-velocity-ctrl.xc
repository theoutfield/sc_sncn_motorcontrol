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


#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


void init_hall_ethercat(hall_par &hall_params, chanend coe_out)
{
	{hall_params.pole_pairs, hall_params.gear_ratio} = hall_sdo_update(coe_out);
}

void init_qei_ethercat(qei_par &qei_params, chanend coe_out)
{
	{qei_params.real_counts, qei_params.gear_ratio, qei_params.index} = qei_sdo_update(coe_out);
}

void init_csv_ethercat(csv_par &csv_params, chanend coe_out)
{
	{csv_params.max_motor_speed, csv_params.nominal_current, csv_params.polarity} = csv_sdo_update(coe_out);
	if(csv_params.polarity >= 0)
		csv_params.polarity = 1;
	else if(csv_params.polarity < 0)
		csv_params.polarity = -1;
}

void init_velocity_ctrl_ethercat(ctrl_par &velocity_ctrl_params, chanend coe_out)
{
	{velocity_ctrl_params.Kp_n, velocity_ctrl_params.Ki_n, velocity_ctrl_params.Kd_n} = velocity_sdo_update(coe_out);
	velocity_ctrl_params.Kp_d = 16384;
	velocity_ctrl_params.Ki_d = 16384;
	velocity_ctrl_params.Kd_d = 16384;

	velocity_ctrl_params.Loop_time = 1 * MSEC_STD;  //units - core timer value //CORE 2/1/0 default

	velocity_ctrl_params.Control_limit = 13739; 	//default

	if(velocity_ctrl_params.Ki_n != 0)    			//auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = (velocity_ctrl_params.Control_limit * velocity_ctrl_params.Ki_d)/velocity_ctrl_params.Ki_n ;
	else
		velocity_ctrl_params.Integral_limit = 0;
	return;
}

void xscope_initialise()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}

ctrl_proto_values_t InOut;

int get_target_velocity()
{
	return InOut.target_velocity;
}
void send_actual_velocity(int actual_velocity)
{
	InOut.velocity_actual = actual_velocity;
}


void ether_comm(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall_p4,chanend c_qei_p4,chanend c_adc,chanend c_torque_ctrl,chanend c_velocity_ctrl,chanend c_position_ctrl)
{
	int i = 0;
	int mode = 2;
	int core_id = 0;

	int target_velocity;
	int actual_velocity = 0;

	timer t;

	int init = 0;
	int flag = 0;

	csv_par 	csv_params;
	ctrl_par 	velocity_ctrl_params;
	qei_par 	qei_params;
	hall_par 	hall_params;
	int sensor_select;

	unsigned int time;
	int state;
	int statusword;
	int controlword;

	int mode_selected = 0;
	check_list checklist;

	state 		= init_state(); 			//init state
	checklist 	= init_checklist();
	InOut 		= init_ctrl_proto();

	init_csv_param(csv_params);

#ifdef ENABLE_xscope_main
	xscope_initialise();
#endif
t:>time;
	while(1)
	{
		ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);

	//	wait_ms(1, core_id, t);
		controlword = InOut.control_word;
		update_checklist(checklist, mode, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
	//	printintln(controlword);

		state = get_next_state(state, checklist, controlword);
		statusword = update_statusword(statusword, state);
		InOut.status_word = statusword;


		//if(state == 3)
		{

		}
		if(mode_selected == 0)
		{
			switch(InOut.operation_mode)
			{
				case CSV: 	//csv mode index
					//printstrln("CSV");
					if(flag == 0)
					{
						init = init_velocity_control(c_velocity_ctrl); //init==1
					}
					if(init == 1)
					{
						flag = 1; mode = 4;

						mode_selected = 1;
						init_velocity_ctrl_ethercat(velocity_ctrl_params, coe_out);
						sensor_select = sensor_select_sdo(coe_out);
						init_csv_ethercat(csv_params, coe_out);
						//init_qei_ethercat(qei_params, coe_out);				//init_hall_ethercat(hall_params, coe_out);
//						printstrln("csv");
//						printintln(velocity_ctrl_params.Kp_n);
//						printintln(velocity_ctrl_params.Ki_n);
//						printintln(velocity_ctrl_params.Kd_n);
						set_velocity_ctrl_ethercat(velocity_ctrl_params, c_velocity_ctrl);
						set_velocity_sensor_ethercat(sensor_select, c_velocity_ctrl);
						InOut.operation_mode_display = CSV;
					}
					break;
			}
		}

		if(mode_selected == 1)
		{
			switch(InOut.control_word)
			{
				case 0x000b: //quick stop

					break;

				case 0x000f: //switch on cyclic
					//printstrln("cyclic");
					target_velocity = get_target_velocity();
					set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);

					actual_velocity = get_velocity(c_velocity_ctrl);
					send_actual_velocity(actual_velocity);

#ifdef ENABLE_xscope_main
					xscope_probe_data(0, actual_velocity);
					xscope_probe_data(1, target_velocity);
#endif
					break;

				case 0x0006: //shutdown

					break;

			}
		}
		t when timerafter(time + MSEC_STD) :> time;

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
	chan c_velocity_ctrl, c_torque_ctrl, c_position_ctrl;

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
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[0] :
		{
			ether_comm(pdo_out, pdo_in, coe_out, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
			// test CSV over ethercat with PVM on master side
		}
		on stdcore[1]:
		{
			//profile_velocity_test(c_signal, c_velocity_ctrl);			// test PVM on slave side

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
							 qei_params, 1, c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
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
