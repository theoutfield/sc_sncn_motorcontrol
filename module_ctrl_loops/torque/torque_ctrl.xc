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
#include "hall_qei.h"
#include <drive_config.h>
#include <internal_config.h>
#include <dc_motor_config.h>
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include "qei_client.h"
#include "hall_qei.h"
#include <internal_config.h>

//#define ENABLE_xscope_torq
#pragma once
#define TORQUE_CTRL_READ(x)		c_torque_ctrl :> x
#define TORQUE_CTRL_WRITE(x)	c_torque_ctrl <: x
#define TORQUE_CTRL_ENABLE()	c_torque_ctrl <: 1
#define TORQUE_CTRL_DISABLE()	c_torque_ctrl <: 0

#define HALL 					1
#define QEI 					2

#define SET_TORQUE_TOKEN 		40
#define GET_TORQUE_TOKEN 		41
#define SET_CTRL_PARAMETER 		101
#define SENSOR_SELECT      		151
#define SHUTDOWN_TORQUE	 		201
#define ENABLE_TORQUE			251


int root_function(int arg);

int init_torque_control(chanend c_torque_ctrl)
{
	int init_state = INIT_BUSY;
	TORQUE_CTRL_ENABLE(); 					//signal torque ctrl loop

	// init check from torque control loop

	while(1)
	{
		init_state = __check_torque_init(c_torque_ctrl);
		//t when timerafter(time+2*MSEC_STD) :> time;
		if(init_state == INIT)
		{
//#ifdef debug_print
			//printstrln("torque intialized");
//#endif
			break;
		}
	}
	return init_state;
}

int get_torque(cst_par &cst_params, chanend c_torque_ctrl)
{
	int torque;
	TORQUE_CTRL_WRITE(GET_TORQUE_TOKEN);
	TORQUE_CTRL_READ(torque);
	return (torque*cst_params.motor_torque_constant);
}

void set_torque(int torque,  cst_par &cst_params, chanend c_torque_ctrl)
{
	torque = torque/cst_params.motor_torque_constant;
	TORQUE_CTRL_WRITE(SET_TORQUE_TOKEN);
	TORQUE_CTRL_WRITE(torque);
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

void send_torque_init_state(chanend c_torque_ctrl, int init_state)
{
	int command;
	select
	{
		case c_torque_ctrl:> command:
			if(command == CHECK_BUSY)
			{
				c_torque_ctrl <: init_state;
			}
			break;
	}
}

int torque_limit(int torque, int max_torque_limit)
{
	if(torque > max_torque_limit) //adc range // (5 * DC900_RESOLUTION)/2
	{
		return max_torque_limit;
	}
	else if(torque < 0 - max_torque_limit)
	{
		return (0 - max_torque_limit);
	}
	else if(torque >= -max_torque_limit && torque <= max_torque_limit)
	{
		return torque;
	}
}

void set_torque_cst(cst_par &cst_params, int target_torque, int torque_offset, chanend c_torque_ctrl)
{
	set_torque( torque_limit( (target_torque + torque_offset) * cst_params.polarity ,	\
			cst_params.max_torque), cst_params , c_torque_ctrl);
}

void set_torque_ctrl_param(ctrl_par &torque_ctrl_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_CTRL_PARAMETER);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kp_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kp_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Ki_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Ki_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kd_n);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Kd_d);
	TORQUE_CTRL_WRITE(torque_ctrl_params.Integral_limit);
}

void init_torque_ctrl_hall(hall_par &hall_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_CTRL_HALL);
	TORQUE_CTRL_WRITE(hall_params.gear_ratio);
	TORQUE_CTRL_WRITE(hall_params.pole_pairs);
}

void init_torque_ctrl_qei(qei_par &qei_params, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SET_TORQUE_CTRL_QEI);
	TORQUE_CTRL_WRITE(qei_params.gear_ratio);
	TORQUE_CTRL_WRITE(qei_params.index);
	TORQUE_CTRL_WRITE(qei_params.real_counts);
	TORQUE_CTRL_WRITE(qei_params.max_count);
	TORQUE_CTRL_WRITE(qei_params.poles);
}
void init_torque_sensor(int sensor_used, chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SENSOR_SELECT);
	TORQUE_CTRL_WRITE(sensor_used);
}

void enable_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(ENABLE_TORQUE);
	TORQUE_CTRL_WRITE(0);
}

void shutdown_torque_ctrl(chanend c_torque_ctrl)
{
	TORQUE_CTRL_WRITE(SHUTDOWN_TORQUE);
	TORQUE_CTRL_WRITE(1);
}

void current_filter(chanend c_adc, chanend c_current, chanend c_speed)
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


	int speed = 0 ;
	//int sensor_select = HALL;

	int tim1, tim2, tim3;
	int adc_calib_start = 0;
	hall_par hall_params;
	init_hall_param(hall_params);

	while(1)
	{
		select
		{
			case c_current :> command:
				//printstrln("start adc calibration");
				adc_calib_start = 1;
				break;
		}
		if(adc_calib_start == 1)
			break;
	}

	do_adc_calibration_ad7949(c_adc);

	c_current<:1; // adc calib done
	init_buffer(buffer_phase_a, filter_length);
	init_buffer(buffer_phase_b, filter_length);
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
		//	xscope_probe_data(0, phase_a_filtered);
		//	xscope_probe_data(1, phase_b_filtered);

			filter_count++;
			if(filter_count == 10)
			{
				filter_count = 0;

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
			break;

//			case tc when timerafter(time1+MSEC_STD+250) :> time1: // .05 ms
//				c_speed <: 2;
//				master{	c_speed :> actual_speed; }
//				break;

			case c_current :> command:
				c_current :> actual_speed;
				c_current <: phase_a_filtered;
				c_current <: phase_b_filtered;
				break;
		}
	}
}



void _torque_ctrl(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		chanend c_current, chanend c_speed, chanend c_commutation, \
		chanend c_hall, chanend c_qei, chanend c_torque_ctrl)
{
	#define filter_dc 80 //80 27
	int actual_speed = 0;
	int command;

	timer tc;
	unsigned int time;
	unsigned int time1;
	int phase_a_filtered = 0;
	int phase_b_filtered = 0;

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

	int i1 = 0;
	int j1 = 0;
	int mod1 = 0;

	int iq_filtered = 0;
	int id_filtered = 0;
	int fdc = filter_dc;
	int fil_cnt_dc = 0;
	int fldc = filter_dc/hall_params.pole_pairs;
	int speed = 0 ;

	int actual_torque = 0;
	int target_torque = 0;
	int absolute_torque = 0;

	int error_torque = 0;
	int error_torque_integral = 0;
	int error_torque_derivative = 0;
	int error_torque_previous = 0;
	int torque_control_output = 0;

	unsigned int received_command = 0;
	int init_state = INIT_BUSY;
	int commutation_init = INIT_BUSY;

	int sensor_used = HALL;

	int qei_counts_per_hall ;
	qei_velocity_par qei_velocity_params;
	int qei_velocity = 0;
	int start_flag = 0;
	int offset_fw_flag = 0;
	int offset_bw_flag = 0;

	int dum, dirn;
	int deactivate = 0;
	int activate = 0;

	init_qei_velocity_params(qei_velocity_params);

	fldc = filter_dc/hall_params.pole_pairs;
	if(fldc < 10)
		fldc = 10;
	qei_counts_per_hall= qei_params.real_counts/ hall_params.pole_pairs;
	init_buffer(buffer_Id, filter_dc);
	init_buffer(buffer_Iq, filter_dc);


	while(1)
	{
		int received_command = UNSET;
		select
		{
			case TORQUE_CTRL_READ(command):
				if(command == SET)
				{
					activate = SET;
					received_command = SET;
//#ifdef debug_print
				//	printstrln("torque activated");
//#endif
				}
				else if(command == UNSET)
				{
					activate = UNSET;
					received_command = SET;
//#ifdef debug_print
					//printstrln("torque disabled");
//#endif
				}
				else if(command == CHECK_BUSY)
				{
					TORQUE_CTRL_WRITE(init_state);
				}
				break;

			default:
				break;
		}
		if(received_command == SET)
		{
			break;
		}
	}

	while(activate)
	{
		if(commutation_init == INIT_BUSY)
		{
		// printstrln("initialized commutation check");
			 commutation_init = __check_commutation_init(c_commutation);
			 if(commutation_init == INIT)
			 {
				 c_current <: 1;
				 break;
			 }
		}
		//send_torque_init_state( c_torque_ctrl,  init_state);
	}
	while(activate)
	{
		#pragma ordered
		select
		{
			case c_current :> command:
				//printstrln("adc calibrated");
				start_flag = 1;
				break;
			case c_torque_ctrl:> command:
				if(command == CHECK_BUSY)
				{
					c_torque_ctrl <: init_state;
				}
				break;
		}
		//send_torque_init_state( c_torque_ctrl,  init_state);
		if(start_flag == 1)
			break;
	}

	init_state = INIT;
	tc :> time1;
	while(activate)
	{
		#pragma ordered
		select
		{
//			case c_speed :> command:
//				slave{c_speed <: actual_speed;}
//				break;

			case tc when timerafter(time1 + MSEC_STD - 100) :> time1:


				if(sensor_used == HALL)
				{
					angle = get_hall_position(c_hall) >> 2; //  << 10 ) >> 12
					actual_speed = get_hall_velocity(c_hall, hall_params);
					{dum, dirn} = get_hall_position_absolute(c_hall);
//					select
//					{
//						case c_speed :> command:
//							slave{c_speed <: actual_speed;}
//							break;
//					}
				}
				else if(sensor_used == QEI)
				{
					//angle = (get_sync_position ( sync_output ) <<10)/qei_counts_per_hall; //synced input old
					{angle, offset_fw_flag, offset_bw_flag} = get_qei_sync_position(c_qei);
					angle = (angle <<10)/qei_counts_per_hall;
					actual_speed = get_qei_velocity( c_qei, qei_params, qei_velocity_params);//
					{dum, dirn} = get_qei_position_absolute(c_qei);

//					select
//					{
//						case c_speed :> command:
//							slave{c_speed <: actual_speed;}
//							break;
//					}

				}

				c_current <: 2;
				c_current <: actual_speed;
				c_current :> phase_a_filtered;
				c_current :> phase_b_filtered;

				phase_1 = 0 - phase_a_filtered;
				phase_2 = 0 - phase_b_filtered;

				#ifdef ENABLE_xscope_torq
				xscope_probe_data(0, phase_a_filtered);
				#endif
				//				xscope_probe_data(1, phase_b_filtered);
				alpha = phase_1;
				beta = (phase_1 + 2*phase_2); 			// beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
				beta *= 37838;
				beta /= 65536;
				beta = -beta;

				// ==== Park transform ====

				sin = sine_table_expanded(angle);
				cos = sine_table_expanded((256 - angle)&1023);

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

				actual_torque = root_function(iq_filtered * iq_filtered + id_filtered * id_filtered);

				#ifdef ENABLE_xscope_torq
				xscope_probe_data(1, actual_torque);
				#endif


				absolute_torque = target_torque;
				if(target_torque < 0)
					absolute_torque = 0 - target_torque;

				error_torque = absolute_torque - actual_torque; //350
				error_torque_integral = error_torque_integral + error_torque;
				error_torque_derivative = error_torque - error_torque_previous;

				if(error_torque_integral > torque_ctrl_params.Integral_limit)
				{
					error_torque_integral = torque_ctrl_params.Integral_limit;
				}
				else if(error_torque_integral < 0-torque_ctrl_params.Integral_limit)
				{
					error_torque_integral = 0 - torque_ctrl_params.Integral_limit;
				}


				torque_control_output = (torque_ctrl_params.Kp_n * error_torque)/torque_ctrl_params.Kp_d\
						+ (torque_ctrl_params.Ki_n * error_torque_integral)/torque_ctrl_params.Ki_d\
						+ (torque_ctrl_params.Kd_n  * error_torque_derivative)/torque_ctrl_params.Kd_d;

				error_torque_previous = error_torque;


				if(target_torque >=0)
				{
					if(torque_control_output >= torque_ctrl_params.Control_limit) {
						torque_control_output = torque_ctrl_params.Control_limit;
					}
					else if(torque_control_output < 0){
						torque_control_output = 0;
					}
				}
				else
				{


					torque_control_output = 0 - torque_control_output;
					if(torque_control_output > 0)
						torque_control_output = 0;
					if(torque_control_output <= -torque_ctrl_params.Control_limit) {
						torque_control_output = 0 - torque_ctrl_params.Control_limit;
					}
				}

				if(!deactivate)
					set_commutation_sinusoidal(c_commutation, torque_control_output);
				else
					set_commutation_sinusoidal(c_commutation, 0);

				break;

			case c_torque_ctrl:> command:
				if(command == SET_TORQUE_TOKEN)
				{
					//c_torque_ctrl :> target_torque;
					TORQUE_CTRL_READ(target_torque);
					#ifdef ENABLE_xscope_torq
					xscope_probe_data(2, target_torque);
					#endif
				}
				else if(command == GET_TORQUE_TOKEN)
				{
					if(torque_control_output >= 0)
						TORQUE_CTRL_WRITE(actual_torque);
					//c_torque_ctrl <: actual_torque;
					else
						//c_torque_ctrl <: (0 - actual_torque);
						TORQUE_CTRL_WRITE(0-actual_torque);
				}
				else if(command == CHECK_BUSY)
				{
					//c_torque_ctrl <: init_state;
					TORQUE_CTRL_WRITE(init_state);
				}
				else if(command == SET_CTRL_PARAMETER)
				{
					TORQUE_CTRL_READ(torque_ctrl_params.Kp_n);
					TORQUE_CTRL_READ(torque_ctrl_params.Kp_d);
					TORQUE_CTRL_READ(torque_ctrl_params.Ki_n);
					TORQUE_CTRL_READ(torque_ctrl_params.Ki_d);
					TORQUE_CTRL_READ(torque_ctrl_params.Kd_n);
					TORQUE_CTRL_READ(torque_ctrl_params.Kd_d);
					TORQUE_CTRL_READ(torque_ctrl_params.Integral_limit);
				}
				else if(command == SENSOR_SELECT)
				{
					TORQUE_CTRL_READ(sensor_used);
				}

				else if(command == SHUTDOWN_TORQUE)
					TORQUE_CTRL_READ(deactivate);

				else if(command == ENABLE_TORQUE)
					TORQUE_CTRL_READ(deactivate);

				else if(command == SET_TORQUE_CTRL_HALL)
				{
					TORQUE_CTRL_READ(hall_params.gear_ratio);
					TORQUE_CTRL_READ(hall_params.pole_pairs);

					fldc =  filter_dc/hall_params.pole_pairs;
					if(fldc < 10)
						fldc = 10;
				}
				else if(command == SET_TORQUE_CTRL_QEI)
				{
					TORQUE_CTRL_READ(qei_params.gear_ratio);
					TORQUE_CTRL_READ(qei_params.index);
					TORQUE_CTRL_READ(qei_params.real_counts);
					TORQUE_CTRL_READ(qei_params.max_count);
					TORQUE_CTRL_READ(qei_params.poles);
					qei_counts_per_hall = qei_params.real_counts/ qei_params.poles;
				}

				break;
		}
	}
}


void torque_control(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl)
{
	chan c_current, c_speed;
	par
	{
		current_filter(c_adc, c_current, c_speed);
		_torque_ctrl(torque_ctrl_params, hall_params, qei_params, c_current, c_speed, c_commutation, c_hall, c_qei, c_torque_ctrl);
	}
}

