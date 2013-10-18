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
#include "hall_qei.h"
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "adc_ad7949.h"
#include "test.h"
#include "comm_loop.h"
#include "refclk.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include <torque_ctrl.h>
#include <flash_somanet.h>
#include <internal_config.h>
#include <drive_config.h>
int root_function(int arg);
#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define HALL 1
#define QEI 2

void init_buffer1(int buffer[], int length)
{
	int i;
	for(i = 0; i < length; i++)
	{
		buffer[i] = 0;
	}
	return;
}


/*	{
			int command;
			int init = 0;
			timer t;
			unsigned int time;
			while(1)
			{
				init = __check_commutation_init(c_signal);
				if(init == INIT)
				{
					printstrln("commutation intialized");
					break;
				}
			}
			init = 1;

			while (1)
			{
				init = __check_torque_init(c_torque_ctrl);
				if(init == INIT)
				{
					printstrln("torque control intialized");
					break;
				}
			}
			//set_torque_test(c_torque_ctrl);//
			while(1)
			{
				t when timerafter(time+2*MSEC_STD) :> time;
				set_torque(c_torque_ctrl, 450);
			}
			//while(1);
		}*/

int main(void)
{
	chan c_adc, c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan sync_output;
	chan c_pwm_ctrl;
	chan dummy, dummy1, dummy2;
	chan c_signal_adc;
	chan c_sig_1, c_signal, c_sync;
	chan c_torque_ctrl, signal_ctrl, c_calib, c_req, c_vel;

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

	chan c_test_in;
	//
	par
	{
		on stdcore[0]:
		{
			par
			{


		{
				#define filter_length 80
				int phase_a_raw = 0;
				int phase_b_raw = 0;
				int actual_speed = 0;
				int command;
				int buffer_phase_a[filter_length];
				int	buffer_phase_b[filter_length];
				timer ts, tc;
				unsigned int time;
				unsigned int time1;
				unsigned int time2;
				int fil_cnt = 0;
				int phase_a_filtered = 0;
				int phase_b_filtered = 0;
				int i = 0;
				int fl = filter_length;
				int j = 0;
				int flc = 3;
				int mod = 0;
				int mod_speed = 0;
				int filter_count = 0;

				// Torque control variables
				int speed = 0 , commutation_init =INIT_BUSY , received_command = 0;
				int sensor_select = QEI;

				int tim1, tim2, tim3;
				int cmdd;
				hall_par hall_params;
				init_hall_param(hall_params);

				while(1)
				{
					if(commutation_init == INIT_BUSY)
					{
			//		 printstrln("initialized commutation check");
						 commutation_init = __check_commutation_init(c_commutation_p1);
						 if(commutation_init == INIT)
						 {
				//			 printstrln("initialized commutation checked");
							 do_adc_calibration_ad7949(c_adc);
							 break;
						 }
					 }
				}
				c_req<:1;
				init_buffer1(buffer_phase_a, filter_length);
				init_buffer1(buffer_phase_b, filter_length);
				ts :> time;
				tc :> time1;

				while(1)
				{
					#pragma ordered
					select
					{
						case ts when timerafter(time+5556) :> time: // .05 ms
						{phase_a_raw , phase_b_raw}= get_adc_calibrated_current_ad7949(c_adc);
					//	xscope_probe_data(0, phase_a_raw);
						buffer_phase_a[fil_cnt] = phase_a_raw;
						buffer_phase_b[fil_cnt] = phase_b_raw;

						fil_cnt = (fil_cnt+1)%fl;
						phase_a_filtered = 0;
						phase_b_filtered = 0;
						j=0;
						for(i=0; i<flc; i++)
						{
							mod = (fil_cnt - 1 - j)%fl;
							if(mod<0)
							mod = fl + mod;
							phase_a_filtered += buffer_phase_a[mod];
							phase_b_filtered += buffer_phase_b[mod];
							j++;
						}
						phase_a_filtered /= flc;
						phase_b_filtered /= flc;
						xscope_probe_data(0, phase_a_filtered);
					//	xscope_probe_data(1, phase_b_filtered);

						filter_count++;
						if(filter_count == 10)
						{
							filter_count = 0;
							//actual_speed = get_hall_velocity(c_hall_p3, hall_params);
						//	xscope_probe_data(0, actual_speed);
							mod_speed = actual_speed;
							if(actual_speed < 0)
								mod_speed = 0 - actual_speed;

							if(mod_speed <= 100)
							{
								flc = 50;
							}
							else if(mod_speed > 100 && mod_speed <= 800)
							{
								flc = 20;
							}
							else if(mod_speed >= 800)
							{
								flc = 3;
							}
						}//
					//	tc :> tim2;
					//	tim3 = tim2-tim1;//
					//	xscope_probe_data(0, tim3);
						break;

						case tc when timerafter(time1+MSEC_STD+250) :> time1: // .05 ms
							if(sensor_select == HALL)
							{
								actual_speed = get_hall_velocity(c_hall_p3, hall_params);
							}
							else if(sensor_select == QEI)
							{
								c_vel <: 2;
								master{	c_vel :> actual_speed; }
							}
							break;

						case c_req :> cmdd:
							c_req <: phase_a_filtered;
							c_req <: phase_b_filtered;
							break;

					}
				}
			}

			{
				#define filter_dc 80 //80 27
				int actual_speed = 0;
				int command;

				timer tc;
				unsigned int time;
				unsigned int time1;
				unsigned int time2;
				int fil_cnt = 0;
				int phase_a_filtered = 0;
				int phase_b_filtered = 0;
				int i = 0;
				int fl = 30;
				int j = 0;
				int flc = 3;
				int mod = 0;
				int mod_speed = 0;
				int filter_count = 0;

				// Torque control variables
				int angle = 0;
				int sin = 0;
				int cos = 0;
				int alpha = 0;
				int beta = 0;
				int Id = 0;
				int Iq = 0;
				int phase_1 = 0;
				int phase_2 = 0;
				int buffer_Id[filter_dc];
				int buffer_Iq[filter_dc];

				int i1 = 0, j1 = 0, mod1 = 0;

				int iq_filtered = 0;
				int id_filtered = 0;
				int fdc = filter_dc;
				int fil_cnt_dc = 0;
				int fldc = filter_dc/POLE_PAIRS;
				int speed = 0 ;

				int torque_actual;

				int torque_target = 1500;

				int torque_error = 0;
				int torque_error_integral = 0;
				int torque_error_derivative = 0;
				int torque_error_previous = 0;
				int torque_control_output = 0;
				const int TORQUE_INTEGRAL_MAX = 137000;
				//int input_torque;
				unsigned int received_command = 0;
				int init_state = INIT_BUSY;
				int commutation_init = INIT_BUSY;

				/* PID Controller variables */
				int Kp;							// Proportional gain
				int Ki;							// Integral gain
				int Kd;							// Derivative gain

				int proportional_member = 0;
				int integral_member = 0;
				int derivative_member = 0;

				int TORQUE_OUTPUT_MAX = 13739;

				//int init_comm = 1;


				int tim1, tim2, tim3;
				int sensor_select = QEI;
				int cmd;
				qei_par qei_params;
				hall_par hall_params;
				int qei_counts_per_hall ;
				qei_velocity_par qei_velocity_params;
				int qei_velocity = 0;
				init_qei_velocity_params(qei_velocity_params);
				init_hall_param(hall_params);
				init_qei_param(qei_params);

				qei_counts_per_hall= qei_params.real_counts/ hall_params.pole_pairs;
				init_buffer1(buffer_Id, filter_dc);
				init_buffer1(buffer_Iq, filter_dc);

				Kp = 15; Kd = 1; Ki = 11;

				tc :> time1;
				select
				{
					case c_req :> cmd:
						printstrln("adc calibrated");
						break;
				}
			//	set_commutation_sinusoidal(c_commutation_p2, -1400);
				while(1)
				{
					#pragma ordered
					select
					{
						case c_vel :> cmd:
							slave{c_vel <: qei_velocity;}
							break;

						case tc when timerafter(time1 + 1*MSEC_STD) :> time1:
							//tc when timerafter(time1+MSEC_STD) :> time1;

							c_req <: 2;
							c_req :> phase_a_filtered;
							c_req :> phase_b_filtered;
						//	ts :> tim1;


							if(sensor_select ==HALL)
							{
								angle = get_hall_position(c_hall_p4) >> 2; //  << 10 ) >> 12
								//angle = (angle*qei_counts_per_hall)>>12;
							}
							else if(sensor_select == QEI)
							{
								angle = (get_sync_position ( sync_output ) <<10)/qei_counts_per_hall; //synced input
								qei_velocity = get_qei_velocity( c_qei_p3, qei_params, qei_velocity_params);//
								select
								{
									case c_vel :> cmd:
										slave{c_vel <: qei_velocity;}
										break;
								}

							}

							phase_1 = 0 - phase_a_filtered;
							phase_2 = 0 - phase_b_filtered;


						//	xscope_probe_data(0, phase_a_filtered);
							//				xscope_probe_data(1, phase_b_filtered);
							alpha = phase_1;
							beta = (phase_1 + 2*phase_2); 			// beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
							beta *= 37838;
							beta /= 65536;
							beta = -beta;

							// ==== Park transform ====

							//theta = angle; 				//range 500 qei
							//theta &= 1023; //499
							sin = sine_table_expanded(angle);//newsine_table[theta]; 		// sine( theta );
						//	theta = (256 - theta)&1023; 				// 90-theta  125
						//	theta &= 1023; 						//499
							cos = sine_table_expanded((256 - angle)&1023); 		// values from 0 to +/- 16384

							Id = ( alpha * cos + beta * sin ) /16384;
							Iq = ( beta * cos  - alpha * sin ) /16384;

							buffer_Id[fil_cnt_dc] = Id;
							buffer_Iq[fil_cnt_dc] = Iq;
							fil_cnt_dc = (fil_cnt_dc+1)%fdc;

							id_filtered = 0;
							iq_filtered = 0;

							j1=0;
							for(i1=0; i1<fldc; i1++)
							{
								mod1 = (fil_cnt_dc - 1 - j1)%fdc;
								if(mod1<0)
								mod1 = fdc + mod1;
								id_filtered += buffer_Id[mod1];
								iq_filtered += buffer_Iq[mod1];
								j1++;
							}
							id_filtered /= fldc;
							iq_filtered /= fldc;

							torque_actual = root_function(iq_filtered * iq_filtered + id_filtered * id_filtered);
							xscope_probe_data(1, torque_actual);
							#ifdef ENABLE_xscope_torq
							xscope_probe_data(0, torque_actual);
							xscope_probe_data(1, torque_target);
							#endif


							Kp = 15; Kd = 1; Ki = 11;

							torque_error = 3 * DC900_RESOLUTION - torque_actual; //350
							torque_error_integral = torque_error_integral + torque_error;
							torque_error_derivative = torque_error - torque_error_previous;

							#ifdef ENABLE_xscope_torq
							//xscope_probe_data(2, torque_error);
							#endif

							if(torque_error_integral > TORQUE_INTEGRAL_MAX)
							{
								torque_error_integral = TORQUE_INTEGRAL_MAX;
							}
							else if(torque_error_integral < -TORQUE_INTEGRAL_MAX)
							{
								torque_error_integral = -TORQUE_INTEGRAL_MAX ;
							}

						//	if(torque_error_integral == 0) torque_error_integral = 1;

							proportional_member = (Kp * torque_error)/10;
							integral_member = (Ki * torque_error_integral)/110;
							derivative_member = (Kd * torque_error_derivative)/10;

							torque_control_output = proportional_member + integral_member + derivative_member;

		#ifdef ENABLE_xscope_torq
							//xscope_probe_data(3, integral_member);
							//xscope_probe_data(4, torque_control_output);
		#endif
							torque_error_previous = torque_error;


							if(torque_target >=0)
							{
								if(torque_control_output >= TORQUE_OUTPUT_MAX) {
									torque_control_output = TORQUE_OUTPUT_MAX;
								}
								else if(torque_control_output < 0){
									torque_control_output = 0;
								}
							}
							else
							{
								if(torque_control_output > 0){
									torque_control_output = -torque_control_output;
								}
								if(torque_control_output <= -TORQUE_OUTPUT_MAX) {
									torque_control_output = 0 - TORQUE_OUTPUT_MAX;
								}
								//else
							}

						//	printstrln("loop");
							set_commutation_sinusoidal(c_commutation_p2, torque_control_output);

						//	speed = qei_velocity;

						//	if(speed<0)
						//		speed = 0 -speed;
						//	if(speed<=370)
						//	{
							//	fldc = 27; //40
						/*	}

							else if( speed > 370 && speed <=1000)
							{
								fldc = 27; //30
							}
							else
							{
								fldc = 20;
							}*/
						//	ts :> tim2;
						//	tim3 = tim2-tim1;//
						//	xscope_probe_data(0, tim3);
							break;
					}
				}
			}
			}
		}


		on stdcore[1]:
		{
			xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
					XSCOPE_DISCRETE, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
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
			par
			{
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED);
					hall_qei_sync(qei_params, hall_params, commutation_params, c_qei_p1, c_hall_p2, sync_output, c_calib);
				}

			/*	{
					hall_par hall_params;
					init_hall_param(hall_params);
					current_ctrl_loop(hall_params, c_signal_adc, c_adc, c_hall_p3,
							sync_output, c_commutation_p1, c_torque_ctrl);
				}*/


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
					int sensor_select = 1;
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); // initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p2, c_signal_adc,
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
