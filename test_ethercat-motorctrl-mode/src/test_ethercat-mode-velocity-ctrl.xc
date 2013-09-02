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
#include <position_ctrl.h>
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

void update_hall_param_ecat(hall_par &hall_params, chanend coe_out)
{
	{hall_params.pole_pairs, hall_params.gear_ratio} = hall_sdo_update(coe_out);
}

void update_qei_param_ecat(qei_par &qei_params, chanend coe_out)
{
	{qei_params.real_counts, qei_params.gear_ratio, qei_params.index} = qei_sdo_update(coe_out);
	qei_params.max_count = __qei_max_counts(qei_params.real_counts);
}

void update_csv_param_ecat(csv_par &csv_params, chanend coe_out)
{
	{csv_params.max_motor_speed, csv_params.nominal_current, csv_params.polarity} = csv_sdo_update(coe_out);
	if(csv_params.polarity >= 0)
		csv_params.polarity = 1;
	else if(csv_params.polarity < 0)
		csv_params.polarity = -1;
}

void update_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend coe_out)
{
	{velocity_ctrl_params.Kp_n, velocity_ctrl_params.Ki_n, velocity_ctrl_params.Kd_n} = velocity_sdo_update(coe_out);
	velocity_ctrl_params.Kp_d = 16384;
	velocity_ctrl_params.Ki_d = 16384;
	velocity_ctrl_params.Kd_d = 16384;

	velocity_ctrl_params.Loop_time = 1 * MSEC_STD;  //units - core timer value //CORE 2/1/0 default

	velocity_ctrl_params.Control_limit = 13739; 	//default

	if(velocity_ctrl_params.Ki_n != 0)    			//auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = velocity_ctrl_params.Control_limit * (velocity_ctrl_params.Ki_d/velocity_ctrl_params.Ki_n) ;
	else
		velocity_ctrl_params.Integral_limit = 0;
	return;
}
void update_position_ctrl_param_ecat(ctrl_par &position_ctrl_params, chanend coe_out)
{
	{position_ctrl_params.Kp_n, position_ctrl_params.Ki_n, position_ctrl_params.Kd_n} = position_sdo_update(coe_out);
	position_ctrl_params.Kp_d = 16384;
	position_ctrl_params.Ki_d = 16384;
	position_ctrl_params.Kd_d = 16384;

	position_ctrl_params.Loop_time = 1 * MSEC_STD;  //units - core timer value //CORE 2/1/0 default

	position_ctrl_params.Control_limit = 13739; 	//default

	if(position_ctrl_params.Ki_n != 0)    			//auto calculated using control_limit
		position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n) ;
	else
		position_ctrl_params.Integral_limit = 0;
	return;
}

void update_csp_param_ecat(csp_par &csp_params, chanend coe_out)
{
	{csp_params.base.max_motor_speed, csp_params.base.polarity, csp_params.base.nominal_current, \
		csp_params.min_position_limit, csp_params.max_position_limit} = csp_sdo_update(coe_out);
	if(csp_params.base.polarity >= 0)
		csp_params.base.polarity = 1;
	else if(csp_params.base.polarity < 0)
		csp_params.base.polarity = -1;
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
int get_target_position()
{
	return InOut.target_position;
}
void send_actual_velocity(int actual_velocity)
{
	InOut.velocity_actual = actual_velocity;
}
void send_actual_position(int actual_position)
{
	InOut.position_actual = actual_position;
}


void ether_comm(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall_p4, \
		chanend c_qei_p4,chanend c_adc,chanend c_torque_ctrl,chanend c_velocity_ctrl,chanend c_position_ctrl)
{
	int i = 0;
	int mode=40;
	int core_id = 0;
	int steps;

	int target_velocity;
	int actual_velocity = 0;
	int target_position;
	int actual_position = 0;

	timer t;

	int init = 0;
	int op_set_flag = 0;
	int op_mode = 0;

	csv_par 	csv_params;
	ctrl_par 	velocity_ctrl_params;
	qei_par 	qei_params;
	hall_par 	hall_params;
	ctrl_par position_ctrl_params;
	csp_par csp_params;

	int ack;
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
 //xscope_initialise();
#endif
t:>time;
	while(1)
	{
		ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);

	//	wait_ms(1, core_id, t);
		controlword = InOut.control_word;
		update_checklist(checklist, mode, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
		//printintln(controlword);

		state = get_next_state(state, checklist, controlword);
		statusword = update_statusword(statusword, state, ack);
		InOut.status_word = statusword;


		//if(state == 3)
		{

		}
		if(mode_selected == 0)
		{
			switch(InOut.operation_mode)
			{
				case CSP:
				//	if(op_set_flag == 0)
					printstrln("CSP");
					if(op_set_flag == 0)
					{
						init = init_position_control(c_position_ctrl); //init==1
					}
					if(init == INIT)
					{
						op_set_flag = 1;
						//enable_velocity_ctrl(c_velocity_ctrl);
						mode_selected = 1;
						op_mode = CSP;
						ack = 0;

						update_position_ctrl_param_ecat(position_ctrl_params, coe_out);
						sensor_select = sensor_select_sdo(coe_out);
						update_csp_param_ecat(csp_params, coe_out);

//						printintln(position_ctrl_params.Control_limit);						printintln(position_ctrl_params.Integral_limit);
//						printintln(position_ctrl_params.Kd_d);						printintln(position_ctrl_params.Ki_d);
//						printintln(position_ctrl_params.Kp_d);						printintln(position_ctrl_params.Kp_n);
//						printintln(position_ctrl_params.Ki_n);						printintln(position_ctrl_params.Kd_n);
//						printintln(position_ctrl_params.Loop_time);
//						printintln(csp_params.base.max_motor_speed);						printintln(csp_params.base.nominal_current);
//						printintln(csp_params.base.polarity);						printintln(csp_params.max_position_limit);
//						printintln(csp_params.min_position_limit);
//						printintln(sensor_select);

//						init_position_ctrl_param_ecat(position_ctrl_params, c_position_ctrl);
//						init_position_sensor_ecat(sensor_select, c_position_ctrl);
						InOut.operation_mode_display = CSP;
					}
					break;

				case CSV: 	//csv mode index
					//printstrln("CSV");
					if(op_set_flag == 0)
					{
						init = init_velocity_control(c_velocity_ctrl); //init==1

					}
					if(init == 1)
					{
						op_set_flag = 1;
						enable_velocity_ctrl(c_velocity_ctrl);
						mode_selected = 1;
						op_mode = CSV;
						ack = 0;

						update_velocity_ctrl_param_ecat(velocity_ctrl_params, coe_out);  //after checking init go to set display mode
						sensor_select = sensor_select_sdo(coe_out);
//
//						if(sensor_select == HALL)
//						{
//							printstrln("HALL");
//							update_hall_param_ecat(hall_params, coe_out);
//							printintln(hall_params.gear_ratio);
//							printintln(hall_params.pole_pairs);
//						}
//						else if(sensor_select == QEI_INDEX || sensor_select == QEI_NO_INDEX)
//						{
//							printstrln("QEI");
//							update_qei_param_ecat(qei_params, coe_out);
//							printintln(qei_params.gear_ratio);
//							printintln(qei_params.index);
//							printintln(qei_params.real_counts);
//							printintln(qei_params.max_count);
//						}
						update_csv_param_ecat(csv_params, coe_out);
//						printintln(velocity_ctrl_params.Kp_n);	printintln(velocity_ctrl_params.Ki_n);
//						printintln(velocity_ctrl_params.Kd_n);



						init_velocity_ctrl_param_ecat(velocity_ctrl_params, c_velocity_ctrl);
						init_velocity_sensor_ecat(sensor_select, c_velocity_ctrl);
						InOut.operation_mode_display = CSV;
					}
					break;

			}
		}
		//printhexln(InOut.control_word);
		if(mode_selected == 1)
		{
			switch(InOut.control_word)
			{
				case 0x000b: //quick stop
					if(op_mode == CSV)
					{
					//printstrln("quick stop");
						actual_velocity = get_velocity(c_velocity_ctrl);//p
						steps = init_quick_stop_velocity_profile(actual_velocity, 1000);//default acc
//
//					//quick_stop_velocity_profile_generate( step);
						i = 0;
						mode_selected = 3;// non interruptible mode
					}
					break;

				case 0x000f: //switch on cyclic
					//printstrln("cyclic");
					if(op_mode == CSV)
					{
						target_velocity = get_target_velocity();	//p
						set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);

						actual_velocity = get_velocity(c_velocity_ctrl);
						send_actual_velocity(actual_velocity);
					}
					else if(op_mode == CSP)
					{
					//	printstrln("cyclic CSP");
						target_position = get_target_position();
						set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);


						actual_position = get_position(c_position_ctrl);
						send_actual_position(actual_position);
					}

#ifdef ENABLE_xscope_main
//					xscope_probe_data(0, actual_velocity);
//					xscope_probe_data(1, target_velocity);
#endif
					break;

				case 0x0006: //shutdown
					//deactivate
					if(op_mode == CSV)
					{
						shutdown_velocity_ctrl(c_velocity_ctrl);//p
						ack = 1;
						op_set_flag = 0; init = 0;
						mode_selected = 0;  // to reenable the op selection and reset the controller
					}
					break;

			}
		}
		if(mode_selected == 3) // non interrupt
		{
			//printintln(mode_selected);
			//printintln(steps);

			if(op_mode == CSV)
			{
				while(i < steps)
				{
					target_velocity = quick_stop_velocity_profile_generate(i);		//p
					set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);
					actual_velocity = get_velocity(c_velocity_ctrl);
#ifdef ENABLE_xscope_main
//					xscope_probe_data(0, actual_velocity);
//					xscope_probe_data(1, target_velocity);
#endif



					t when timerafter(time + MSEC_STD) :> time;
					i++;
				}
				if(i >= steps)
				{
					if(actual_velocity < 50 || actual_velocity > -50)
					{
						//printstrln("stopped");

						state = 2;
						statusword = update_statusword(statusword, state, ack);
						InOut.status_word = statusword;
						ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
						mode_selected = 100;
						op_set_flag = 0; init = 0;
					}
				}
			}


		}
		if(mode_selected ==100)
		{
			ack = 1;
			statusword = update_statusword(statusword, state, ack);
			InOut.status_word = statusword;
			//ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
			switch(InOut.operation_mode)
			{
				case 100: mode_selected = 0;
					ack = 0;
					InOut.operation_mode_display = 100;
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

		on stdcore[1] :
		{
			ether_comm(pdo_out, pdo_in, coe_out, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
			// test CSV over ethercat with PVM on master side
		}

		on stdcore[2]:
		{
			par
			{
				{
					 ctrl_par position_ctrl_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_position_control_param(position_ctrl_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 position_control(position_ctrl_params, hall_params, qei_params, 2, c_hall_p3,\
							 c_qei_p2, c_position_ctrl, c_commutation_p3);
				}

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
