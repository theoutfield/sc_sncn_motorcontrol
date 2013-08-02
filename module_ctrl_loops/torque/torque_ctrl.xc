/**
 * \file torque_ctrl.xc
 *
 *	Torque control rountine based on field oriented Torque control method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "torque_ctrl.h"
#include <refclk.h>
#include <xscope.h>
#include <print.h>
#include "hall-qei.h"
#include <drive_config.h>
//#define ENABLE_xscope_torq
#define defParAngleUser 560
#define filter_length 30
#define filter_dc 160

int root_function(int arg);
int get_torque(chanend c_torque)
{
	int torque;
	c_torque <: 3;
	c_torque :> torque;
	return torque;
}

void set_torque(chanend c_torque, int torque)
{
	c_torque <: 2;
	c_torque <: torque;
	return;
}

void init_buffer(int buffer[], int length)
{
	int i;
	for(i = 0; i < length; i++)
	{
		buffer[i] = 0;
	}
	return;
}

void current_ctrl_loop(hall_par &hall_params, chanend signal_adc, chanend c_adc,
		chanend c_hall, chanend sync_output, chanend c_commutation,	chanend c_torque) {

	int phase_a_raw = 0;
	int phase_b_raw = 0;
	int actual_speed;
	int command;
	int buffer_phase_a[filter_length];
	int	buffer_phase_b[filter_length];

	timer ts;
	timer ts1;
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
	int mod;
	int mod_speed;
	int filter_count = 0;

	/* Torque control variables */
	int hall_angle;
	int sin;
	int cos;
	int alpha;
	int beta;
	int Id;
	int Iq;
	int phase_1 = 0;
	int phase_2 = 0;
	int buffer_Id[filter_dc];
	int buffer_Iq[filter_dc];


	unsigned theta; // angle
	int i1 = 0, j1 = 0, mod1 = 0;

	int iq_filtered = 0;
	int id_filtered = 0;
	int fdc = filter_dc;
	int fil_cnt_dc = 0;
	int fldc = filter_dc;
	int speed = 0;

	int wait = 21000;

	int torque_target = 0;
	int torque_actual;
	int torque_error = 0;
	int torque_error_integral = 0;
	int torque_error_derivative = 0;
	int torque_error_previous = 0;
	int torque_control_output = 0;
	const int TORQUE_INTEGRAL_MAX = 137000;
	int input_torque;

	int init_state = INIT_BUSY;

	/* PID Controller variables */
	int Kp;							// Proportional gain
	int Ki;							// Integral gain
	int Kd;							// Derivative gain

	int proportional_member = 0;
	int integral_member = 0;
	int derivative_member = 0;

	int TORQUE_OUTPUT_MAX = 13739;

	//int init_comm = 1;
	Kp = 15; Kd = 1; Ki = 11;


/*	init = init_commutation(c_signal);
	if(init == 1)
		printstrln("initialized commutation");
	else
		printstrln(" initialize commutation failed");
*/
	signal_adc <: 1;

	while (1) {
		unsigned received_command = 0;
		select
		{
			case signal_adc :> command:
				do_adc_calibration_ad7949(c_adc);
				received_command =1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
			//printstrln("adc calibrated");
			break;
		}
	}


	init_buffer(buffer_phase_a, filter_length);
	init_buffer(buffer_phase_b, filter_length);
	init_buffer(buffer_Id, filter_dc);
	init_buffer(buffer_Iq, filter_dc);


	//c_torque <: 1;  						//signal outerloop
	init_state = INIT;


	ts :> time;
	tc :> time1;
	ts1 :> time2;

	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time+5556) :> time: // .05 ms
				{phase_a_raw , phase_b_raw}= get_adc_vals_calibrated_int16_ad7949(c_adc);
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
			//	xscope_probe_data(0, phase_a_filtered);
			//	xscope_probe_data(1, phase_b_filtered);

				filter_count++;
				if(filter_count == 10)
				{
					filter_count = 0;
					actual_speed = get_hall_speed(c_hall, hall_params);
				//	xscope_probe_data(0, actual_speed);
					mod_speed = actual_speed;
					if(actual_speed<0)
					mod_speed = 0 - actual_speed;
					if(mod_speed<370)
					{
						flc = 20;
					}
					if(mod_speed < 100)
					{
						flc = 50;
					}
					if(mod_speed>800)
					flc = 3;
				}
			break;


			case tc when timerafter(time1+wait) :> time1:
				wait = 21000;
				hall_angle = get_sync_position ( sync_output ); //synced input

				phase_1 = 0 - phase_a_filtered;
				phase_2 = 0 - phase_b_filtered;

				alpha = phase_1;
				beta = (phase_1 + 2*phase_2); 			// beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
				beta *= 37838;
				beta /= 65536;
				beta = -beta;

				/* ==== Park transform ==== */

				theta = hall_angle; 				//range 500 qei
				theta &= 499;
				sin = newsine_table[theta]; 		// sine( theta );
				theta = (125 - theta); 				// 90-theta
				theta &= 499; 						//499
				cos = newsine_table[theta]; 		// values from 0 to +/- 16384

				Id = (((alpha * cos ) /16384) + ((beta * sin ) /16384));
				Iq = (((beta * cos ) /16384) - ((alpha * sin ) /16384));

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

				#ifdef ENABLE_xscope_torq
				xscope_probe_data(0, torque_actual);
				xscope_probe_data(1, torque_target);
				#endif

				speed = actual_speed;

				if(speed<0)
					speed = 0 -speed;
				if(speed<370)
				{
					fldc = 27; //40
				}

				else if( speed > 370 && speed <1000)
				{
					fldc = 27; //30
				}
				else
				{
					fldc = 20;
				}

				break;

			case ts1 when timerafter(time2+7700) :> time2:

				if(torque_actual > MAX_NOMINAL_CURRENT * DC900_RESOLUTION) //400
				{
					Kp = 15; Kd = 1; Ki = 11;

					torque_error = MAX_NOMINAL_CURRENT * DC900_RESOLUTION - torque_actual; //350
					torque_error_integral = torque_error_integral + torque_error;
					torque_error_derivative = torque_error - torque_error_previous;

#ifdef ENABLE_xscope_torq
					//xscope_probe_data(2, torque_error);
#endif

					if(torque_error_integral > TORQUE_INTEGRAL_MAX)
					{
						torque_error_integral = TORQUE_INTEGRAL_MAX;
					}
					else if(torque_error_integral < 0)
					{
						torque_error_integral = 0 ;
					}

					if(torque_error_integral == 0) torque_error_integral = 1;

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
						if(torque_control_output <= -TORQUE_OUTPUT_MAX) {
							torque_control_output = 0 - TORQUE_OUTPUT_MAX;
						}
						else if(torque_control_output > 0){
							torque_control_output = 0;
						}
					}



					c_commutation <: 2;
					c_commutation <: torque_control_output;
					torque_target = torque_control_output;
				}
				else
				{
					c_commutation <: 2;
					c_commutation <: torque_target;
					torque_error_integral = (torque_target*110)/Ki;
				}

				break;


		}

		select
		{
			case c_torque:> command:
				if(command == 2)
				{
					c_torque :> torque_target;
					if(torque_target > 13700)
					{
						torque_target = 13700;
					}
				}
				if(command == 3)
				{
					c_torque <: torque_actual;
				}
				if(command == CHECK_BUSY)
				{
					c_torque <: init_state;
				}

				break;
			default:
				break;

		}
	}
}


