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
#include <ioports.h>
#include <refclk.h>
#include <pwm_config.h>
#include <pwm_service_inv.h>
#include <sine_table_big.h>
#include <comm_loop_server.h>

#include <hall_server.h>
#include <hall_client.h>
#include <qei_server.h>
#include <qei_client.h>
#include <adc_server_ad7949.h>
#include <adc_client_ad7949.h>
#include <flash_somanet.h>
#include <ctrlproto.h>
#include <velocity_ctrl_server.h>
#include <position_ctrl_server.h>

#include <drive_config.h>
#include <internal_config.h>
#include <bldc_motor_config.h>
#include <comm.h>
#include <xscope.h>
#include <print.h>
#include <test1.h>

int checkout = 0;

#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

//#define print_slave

ctrl_proto_values_t InOut;




int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_home;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl, c_position_ctrl , c_torque_ctrl, c_watchdog;

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

	chan info;
	//
	par
	{
	/*	on stdcore[0] :
		{
			ecat_init();

			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out, foe_in, pdo_out, pdo_in);
		}
		on stdcore[0] :
		{
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}*/

		on stdcore[2] :
		{
			ecat_motor_test( pdo_out,  pdo_in,  coe_out,  c_signal,  c_hall_p1,\
					 c_qei_p1,  c_home,  c_torque_ctrl,  c_velocity_ctrl,  c_position_ctrl);
		}

/*		on stdcore[1]:
		{
			timer t;
			unsigned int time;
			int state;

			int statusword;
			int controlword;
			int mode = 3;
			check_list checklist;

			state 		= init_state(); 			//init state
			checklist 	= init_checklist();
			InOut 		= init_ctrl_proto();


			t :> time;
			while(1)
			{
//				ctrlproto_protocol_handler_function(pdo_out, pdo_in, InOut);
//				t when timerafter(time + MSEC_STD) :> time;

				controlword = InOut.control_word;
				update_checklist(checklist, mode, c_signal, c_hall_p4, c_qei_p4, c_adc, c_torque_ctrl, c_velocity_ctrl, c_position_ctrl);
				printintln(controlword);

				state = get_next_state(state, checklist, controlword);
			//	statusword = update_statusword(statusword, state, 0);
				InOut.status_word = statusword;
			}
		}*/

		on stdcore[3]:
		{


/*			par
			{

				{
					 ctrl_par velocity_ctrl_params;
					 filter_par sensor_filter_params;
					 hall_par hall_params;
					 qei_par qei_params;

					// init_velocity_control_param(velocity_ctrl_params);
					 init_sensor_filter_param(sensor_filter_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, qei_params, 2,\
							 c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
				}


				{
					 ctrl_par position_ctrl_params;
					 hall_par hall_params;
					 qei_par qei_params;

					// init_position_control_param(position_ctrl_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 position_control(position_ctrl_params, hall_params, qei_params, QEI, c_hall_p3,\
							 c_qei_p2, c_position_ctrl, c_commutation_p3);
				}

			}*/
		}
		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
//			par
//			{
//
////				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia,
////						p_ifm_adc_misoa, p_ifm_adc_misob);
//
//				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
//						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
//
//				{
//					hall_par hall_params;
//					qei_par qei_params;
//					commutation_par commutation_params;
//					//init_hall_param(hall_params);
//					//fu(c_signal);
//					commutation_init_ecat(c_signal, hall_params, qei_params, commutation_params);
//					commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, \
//							c_commutation_p1, c_commutation_p2, c_commutation_p3, \
//							c_pwm_ctrl, hall_params, qei_params, commutation_params);					 // hall based sinusoidal commutation
//				}
//
////				{
////					hall_par hall_params;
////					init_hall_param(hall_params);
////					run_hall(p_ifm_hall, hall_params, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4
////				}
////
////				{
////					qei_par qei_params;
////					init_qei_param(qei_params);
////					run_qei(p_ifm_encoder, qei_params, c_qei_p1, c_qei_p2, c_qei_p3 , c_qei_p4);  // channel priority 1,2..4
////				}
//
//			}
		}

	}

	return 0;
}
/* 	{
				int command;
				int init_state = INIT;
				while(1)
				{
					select
					{
						case c_signal :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}
							break;

						case c_position_ctrl :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}

							break;

						case c_torque_ctrl :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}

							break;

						case c_velocity_ctrl :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}
							break;

						case c_qei_p1 :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}
							break;

						case c_hall_p1 :> command:
							if(command == CHECK_BUSY)			// init signal
							{
								c_signal <: init_state;
							}
							break;
					}
				}

			}*/

