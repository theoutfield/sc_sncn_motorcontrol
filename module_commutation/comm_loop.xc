
/**
 * \file comm_loop.xc
 *
 *	Commutation rountine based on Space Vector PWM method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			& Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/


#include "comm_loop.h"
#include <xs1.h>
#include <stdint.h>
#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "predriver/a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include "dc_motor_config.h"
#include <xscope.h>
#include "refclk.h"
#include "qei_client.h"
#include "hall_qei.h"
#include <internal_config.h>
#include "print.h"
#define SET_VOLTAGE    2
#define SET_COMMUTATION_PARAMS 3
#define HALL 1
#define QEI 2
static t_pwm_control pwm_ctrl;


void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed)
{
	commutation_params.angle_variance = 1024/(hall_params.pole_pairs * 3); // (60 * 4096)/( POLE_PAIRS * 2 *360)
	if(hall_params.pole_pairs < 4)
	{
		commutation_params.max_speed_reached = nominal_speed*4;
		commutation_params.flag = 1;
	}
	else if(hall_params.pole_pairs >=4)
	{
		commutation_params.max_speed_reached = nominal_speed;//10000;//
		commutation_params.flag = 0;
	}
	commutation_params.qei_forward_offset = 0;
	commutation_params.qei_backward_offset = 0;
//	printintln(commutation_params.angle_variance);
//	printintln(commutation_params.max_speed_reached);
}
/*
int init_commutation(chanend c_signal)
{
	unsigned command, received_command = 0;

	// init check from commutation loop
	while (1)
	{
		select
		{
			case SIGNAL_READ(command):
				received_command = SUCCESS;
				break;
			default:
				break;
		}
		if(received_command == SUCCESS)
		{
			break;
		}
	}
	return received_command;
}*/

void commutation_init_to_zero(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}


int absolute(int var)
{
	if(var < 0)
		var = 0 - var;
	return var;
}

#define FORWARD_CONSTANT 683  	//60  deg
#define REVERSE_CONSTANT 2731	//240 deg
#define CHANGE_SENSOR 20


void commutation_sensor_select(chanend c_commutation, int sensor_select)
{
	c_commutation <: CHANGE_SENSOR;
	c_commutation <: sensor_select;
	return;
}
/* Sinusoidal based commutation functions */

void commutation_client_hanlder(chanend c_commutation, int command, commutation_par &commutation_params, int &voltage, int &sensor_select, int init_state)
{
	if(command == SET_VOLTAGE)				// set voltage
	{
		c_commutation :> voltage;
		return;
	}
	else if(command == SET_COMMUTATION_PARAMS)
	{
		c_commutation :> commutation_params.angle_variance;
		c_commutation :> commutation_params.max_speed_reached;
		c_commutation :> commutation_params.offset_forward;
		c_commutation :> commutation_params.offset_backward;

		return;
	}
	else if(command == CHANGE_SENSOR)
	{
		c_commutation :> sensor_select;
		return;
	}
	else if(command == CHECK_BUSY)			// init signal
	{
		c_commutation <: init_state;
	}
	return;
}

void commutation_sinusoidal_loop(int sensor_select, hall_par &hall_params, qei_par &qei_params,
		commutation_par &commutation_params, chanend c_hall, chanend c_qei,	chanend c_sync,
		chanend c_pwm_ctrl,	chanend c_signal, chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3)
{
	unsigned int command;
	unsigned int pwm[3] = { 0, 0, 0 };
	int angle_pwm = 0;
	int angle = 0;
	int angle_rpm   = 0;
	int speed = 0;
	timer t;
	unsigned int ts;
	int voltage = 0;
	int direction = 0;
	int init_state = INIT;
	int pwm_half = PWM_MAX_VALUE>>1;
	int max_count_per_hall = qei_params.real_counts/hall_params.pole_pairs;
	int angle_offset = 682/(2*hall_params.pole_pairs);

	int fw_flag = 0;
	int bw_flag = 0;

	qei_velocity_par qei_velocity_params;
	init_qei_velocity_params(qei_velocity_params);

	while (1)
	{
		//xscope_probe_data(0, direction);

		if(sensor_select == HALL) //hall only
		{
			speed = get_hall_velocity(c_hall, hall_params);
			angle = get_hall_position(c_hall);
			angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;
		}
		else if(sensor_select == QEI)
		{
			//angle = (get_sync_position(c_sync) << 12)/max_count_per_hall;
			{angle, fw_flag, bw_flag} = get_qei_sync_position(c_qei);
			angle = (angle << 12)/max_count_per_hall;
			if(voltage >=0)
			{
				if(fw_flag == 0)
				{
					angle = get_hall_position(c_hall);
				}
			}
			else if(voltage <0)
			{
				if(bw_flag == 0)
				{
					angle = get_hall_position(c_hall);
				}
			}
			angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;
		}

		if(voltage<0)
			direction = -1;
		else if(voltage >= 0)
			direction = 1;


		if (direction == 1)
		{
			if(sensor_select == HALL)
			{
				angle_pwm = ((angle + angle_rpm + FORWARD_CONSTANT - commutation_params.angle_variance) & 0x0fff) >> 2;					//100 M3  //100 M1 //180           old 480
			}
			else if(sensor_select == QEI)
			{
				angle_pwm = ((angle + commutation_params.offset_forward) & 0x0fff) >> 2;	 //512
			}
			pwm[0] = ((sine_third_expanded(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;

			pwm[1] = ((sine_third_expanded(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_third_expanded(angle_pwm))*voltage)/13889   + pwm_half;

		}
		else if (direction == -1)
		{
			if(sensor_select == HALL)
			{
				angle_pwm = ((angle - angle_rpm + REVERSE_CONSTANT + commutation_params.angle_variance) & 0x0fff) >> 2;  				//2700 M3  //  2550 M1 //2700      old 3000
			}
			else if(sensor_select == QEI)
			{
				angle_pwm = ((angle  + commutation_params.offset_backward ) & 0x0fff) >> 2;  	 //3100
			}
			pwm[0] = ((sine_third_expanded(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;

			pwm[1] = ((sine_third_expanded(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_third_expanded(angle_pwm))*-voltage)/13889   + pwm_half;

		}

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

#pragma ordered
		select
		{
			case c_commutation_p1 :> command:
				commutation_client_hanlder( c_commutation_p1, command, commutation_params, voltage, sensor_select, init_state);
				break;

			case c_commutation_p2 :> command:
				commutation_client_hanlder( c_commutation_p2, command, commutation_params, voltage, sensor_select, init_state);
				break;

			case c_commutation_p3 :> command:
				commutation_client_hanlder( c_commutation_p3, command, commutation_params, voltage, sensor_select, init_state);
				break;

			case c_signal :> command:
				if(command == CHECK_BUSY)			// init signal
				{
					c_signal <: init_state;
				}
				break;

			default:
				break;
		}
	}

}

void set_commutation_params(chanend c_commutation, commutation_par &commutation_params)
{
	c_commutation <: SET_COMMUTATION_PARAMS;
	c_commutation <: commutation_params.angle_variance;
	c_commutation <: commutation_params.max_speed_reached;
	c_commutation <: commutation_params.offset_forward;
	c_commutation <: commutation_params.offset_backward;
}
/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage)
{
	c_commutation <: 2;
	c_commutation <: input_voltage;
	return;
}

void commutation_sinusoidal(chanend c_hall, chanend c_qei,\
		chanend c_signal, chanend c_sync, chanend  c_commutation_p1, chanend  c_commutation_p2,\
		chanend  c_commutation_p3, chanend c_pwm_ctrl, hall_par &hall_params,\
		qei_par &qei_params, commutation_par &commutation_params)
{
	  const unsigned t_delay = 300*USEC_FAST;
	  const unsigned timeout = 2*SEC_FAST;
	  timer t;
	  unsigned ts;
	  int init_state = INIT_BUSY;


	  commutation_init_to_zero(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	//  printstrln("start");


	//  c_signal <: 1; 			//signal commutation init done.

	  t :> ts;
/*	  while(1)
	  {
		  unsigned command, received_command = 0;
		  #pragma ordered
		  select
		  {
//			case c_signal_adc :> command:
//				received_command = 1;
//				//printstrln("received signal from torque ctrl");
//				c_signal_adc <: 1;
//				break;
//			case t when timerafter(ts + timeout) :> void:
//				received_command = 1;
//				//printstrln("timed out");
//				break;
			case c_signal :> command:  //
				if(command == CHECK_BUSY)
					c_signal <: init_state;
				break;
			default:
				break;
		  }
		  if(received_command == 1)
			  break;
	  }*/

	 // printstrln("start commutation");

	//  if( sensor_select ==  HALL)
		  commutation_sinusoidal_loop(HALL, hall_params, qei_params, commutation_params,\
				  c_hall, c_qei, c_sync, c_pwm_ctrl, c_signal, c_commutation_p1, c_commutation_p2, c_commutation_p3);
	//  else if(sensor_select == QEI)
	//	  commutation_sinusoidal_loop_qei( qei_params,hall_params, commutation_params, c_hall, c_pwm_ctrl, c_signal, c_sync, c_commutation_p1, c_commutation_p2, c_commutation_p3);
}





