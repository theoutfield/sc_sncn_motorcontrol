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
#include <comm.h>

//#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

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
	ctrl_par	position_ctrl_params;
	csp_par 	csp_params;
	pp_par 		pp_params;
	pv_par		pv_params;

	int setup_loop_flag = 0;
	int sense;

	int ack = 0;
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
	init_csp_param(csp_params);
	init_hall_param(hall_params);
#ifdef ENABLE_xscope_main
 xscope_initialise();
#endif
t:>time;
	while(1)
	{
		ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);


		controlword = InOut.control_word;
		update_checklist(checklist, mode, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);

		state = get_next_state(state, checklist, controlword);
		statusword = update_statusword(statusword, state, ack);
		InOut.status_word = statusword;


		if(setup_loop_flag == 0)
		{
			if(controlword == 6)
			{
				update_hall_param_ecat(hall_params, coe_out);
				update_qei_param_ecat(qei_params, coe_out);
				sensor_select = sensor_select_sdo(coe_out);

				set_commutation_param_ecat(c_signal, hall_params);
				set_hall_param_ecat(c_hall_p4, hall_params);
				set_qei_param_ecat(c_qei_p4, qei_params);

				setup_loop_flag = 1;
			}
		}
		if(mode_selected == 0)
		{
			switch(InOut.operation_mode)
			{
				case PP:
					printstrln("ss");
				//	config_sdo_handler(coe_out);
					update_pp_param_ecat(pp_params, coe_out);
					printintln(pp_params.base.max_profile_velocity);
					printintln(pp_params.profile_velocity);
					printintln(pp_params.base.profile_acceleration);
					printintln(pp_params.base.profile_deceleration);
					printintln(pp_params.base.quick_stop_deceleration);
					InOut.operation_mode_display = PP;
					break;

				case PV:
					printstrln("pv");
					update_pv_param_ecat(pv_params, coe_out);
					printintln(pv_params.max_profile_velocity);
					printintln(pv_params.profile_acceleration);
					printintln(pv_params.profile_deceleration);
					printintln(pv_params.quick_stop_deceleration);
					InOut.operation_mode_display = PV;
					break;

				case CSP:
					if(op_set_flag == 0)
					{
						init = init_position_control(c_position_ctrl);
					}
					if(init == INIT)
					{
						op_set_flag = 1;
						enable_position_ctrl(c_position_ctrl);
						mode_selected = 1;
						op_mode = CSP;
						ack = 0;

						update_position_ctrl_param_ecat(position_ctrl_params, coe_out);
						sensor_select = sensor_select_sdo(coe_out);
						update_csp_param_ecat(csp_params, coe_out);
						//printintln(csp_params.base.max_acceleration);
						if(sensor_select == HALL)
						{
							update_hall_param_ecat(hall_params, coe_out);
							init_position_ctrl_hall(hall_params, c_position_ctrl);
						}
						else if(sensor_select == QEI_INDEX || sensor_select == QEI_NO_INDEX)
						{
							update_qei_param_ecat(qei_params, coe_out);
							init_position_ctrl_qei(qei_params, c_position_ctrl);
						}

						init_position_ctrl_param_ecat(position_ctrl_params, c_position_ctrl);
						init_position_sensor_ecat(sensor_select, c_position_ctrl);

						InOut.operation_mode_display = CSP;
					}
					break;

				case CSV: 	//csv mode index
					if(op_set_flag == 0)
					{
						init = init_velocity_control(c_velocity_ctrl);

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
						update_csv_param_ecat(csv_params, coe_out);

						if(sensor_select == HALL)
						{
							update_hall_param_ecat(hall_params, coe_out);
							init_velocity_ctrl_hall(hall_params, c_velocity_ctrl);
						}
						else if(sensor_select == QEI_INDEX || sensor_select == QEI_NO_INDEX)
						{
							update_qei_param_ecat(qei_params, coe_out);
							init_velocity_ctrl_qei(qei_params, c_velocity_ctrl);
						}

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
						actual_velocity = get_velocity(c_velocity_ctrl);
						steps = init_quick_stop_velocity_profile(actual_velocity, 1000);//default acc
						i = 0;
						mode_selected = 3;// non interruptible mode
					}
					else if(op_mode == CSP)
					{
						actual_velocity = get_hall_speed(c_hall_p4, hall_params);
						actual_position = get_position(c_position_ctrl);
						if(!(actual_velocity<40 && actual_velocity>-40))
						{
							if(actual_velocity < 0)
							{
								actual_velocity = 0-actual_velocity;
								sense = -1;
							}
							steps = init_stop( (actual_velocity*360)/(60*hall_params.gear_ratio), actual_position, csp_params.base.max_acceleration);
							i = 0;
							mode_selected = 3;// non interruptible mode
						}
						else
						{
							mode_selected = 100;
							op_set_flag = 0; init = 0;
						}
					}
					break;

				case 0x000f: //switch on cyclic
					//printstrln("cyclic");
					if(op_mode == CSV)
					{
						target_velocity = get_target_velocity();
						set_velocity_csv(csv_params, target_velocity, 0, 0, c_velocity_ctrl);

						actual_velocity = get_velocity(c_velocity_ctrl);
						send_actual_velocity(actual_velocity);
					}
					else if(op_mode == CSP)
					{
						target_position = get_target_position();
						set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);


						actual_position = get_position(c_position_ctrl);
						send_actual_position(actual_position);
#ifdef ENABLE_xscope_main
										xscope_probe_data(0, actual_position);
											xscope_probe_data(1, target_position);
#endif
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
					if(op_mode == CSP)
					{
						shutdown_position_ctrl(c_position_ctrl);//p
						ack = 1;
						op_set_flag = 0; init = 0;
						mode_selected = 0;  // to reenable the op selection and reset the controller
					}
					break;

			}
		}
		if(mode_selected == 3) // non interrupt
		{
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
						state = 2;
						statusword = update_statusword(statusword, state, ack);
						InOut.status_word = statusword;
						ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
						mode_selected = 100;
						op_set_flag = 0; init = 0;
					}
				}
			}
			else if(op_mode == CSP)
			{
				{actual_position, sense} = get_qei_position_count(c_qei_p4);
				while(i < steps)
				{
					target_position   =   mot_q_stop(i, sense);
					set_position_csp(csp_params, target_position, 0, 0, 0, c_position_ctrl);
					actual_position = get_position(c_position_ctrl);
#ifdef ENABLE_xscope_main
					xscope_probe_data(0, actual_position);
					xscope_probe_data(1, target_position);
#endif
					t when timerafter(time + MSEC_STD) :> time;
					i++;
				}
				if(i >=steps )
				{
					actual_velocity = get_hall_speed(c_hall_p4, hall_params);
					if(actual_velocity < 50 || actual_velocity > -50)
					{
						mode_selected = 100;
						op_set_flag = 0; init = 0;
					}
				}
#ifdef ENABLE_xscope_main
										xscope_probe_data(0, actual_position);
											xscope_probe_data(1, target_position);
#endif
			}


		}
		if(mode_selected ==100)
		{
			ack = 1;
			switch(InOut.operation_mode)
			{
				case 100:
					mode_selected = 0;
					ack = 0;
					InOut.operation_mode_display = 100;
					break;
			}
			xscope_probe_data(0, actual_position);
													xscope_probe_data(1, target_position);
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
				//	init_hall_param(hall_params);
					comm_init_ecat(c_signal, hall_params);

					commutation_sinusoidal(hall_params, c_hall_p1, c_pwm_ctrl, c_signal_adc, c_signal,
							c_commutation_p1, c_commutation_p2, c_commutation_p3);					 // hall based sinusoidal commutation
				}

				{
					hall_par hall_params;
				//	init_hall_param(hall_params);
					hall_init_ecat(c_hall_p4, hall_params);

					run_hall(p_ifm_hall, hall_params, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
				//	init_qei_param(qei_params);
					qei_init_ecat(c_qei_p4, qei_params);

					run_qei(p_ifm_encoder, qei_params, c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
