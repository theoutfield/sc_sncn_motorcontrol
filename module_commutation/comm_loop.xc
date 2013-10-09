
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


#include <xs1.h>
#include <stdint.h>
#include <xscope.h>
#include "refclk.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include "qei_client.h"
#include "hall-qei.h"
#include "comm_loop.h"
#include <internal_config.h>
#include "print.h"

#define SET_VOLTAGE    2
#define SET_COMMUTATION_PARAMS 3
#define HALL 1
#define QEI 2
static t_pwm_control pwm_ctrl;

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
}

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
/* Sinusoidal based commutation functions */

void commutation_sinusoidal_loop(int sensor_select, hall_par &hall_params, qei_par &qei_params,
		commutation_par &commutation_params, chanend c_hall, chanend c_qei,	chanend c_pwm_ctrl,
		chanend c_signal, chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3)
{
	unsigned int command;
	unsigned int pwm[3] = { 0, 0, 0 };
	int angle_pwm = 0;
	int angle = 0;
	int angle_rpm   = 0;
	int speed = 0;

	int voltage = 0;
	int direction = 0;
	int init_state = INIT;
	int pwm_half = PWM_MAX_VALUE>>1;


	while (1)
	{
		//xscope_probe_data(0, direction);

		//if(sensor_select == 1)
		{
			speed = get_hall_velocity(c_hall, hall_params);
			angle = get_hall_position(c_hall);
		}


		angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;

		if(voltage<0)
			direction = -1;
		else if(voltage >= 0)
			direction = 1;


		if (direction == 1)
		{
			angle_pwm = ((angle + angle_rpm + FORWARD_CONSTANT - commutation_params.angle_variance) & 0x0fff) >> 2;					//100 M3  //100 M1 //180           old 480
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;
			pwm[1] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;

		}
		else if (direction == -1)
		{
			angle_pwm = ((angle - angle_rpm + REVERSE_CONSTANT + commutation_params.angle_variance) & 0x0fff) >> 2;  				//2700 M3  //  2550 M1 //2700      old 3000
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;

			pwm[1] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;

		}

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

		select
		{
			case c_commutation_p1 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p1 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p1 :> commutation_params.angle_variance;
					c_commutation_p1 :> commutation_params.max_speed_reached;
				}
				break;
			case c_commutation_p2 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p2 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p2 :> commutation_params.angle_variance;
					c_commutation_p2 :> commutation_params.max_speed_reached;
				}
				break;
			case c_commutation_p3 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p3 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p3 :> commutation_params.angle_variance;
					c_commutation_p3 :> commutation_params.max_speed_reached;
				}
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


void commutation_sinusoidal_loop_qei(qei_par &qei_params, hall_par &hall_params,commutation_par &commutation_params,
		chanend c_qei,	chanend c_pwm_ctrl, chanend c_signal, chanend c_sync,
		chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3)
{
	unsigned int command;
	unsigned int pwm[3] = { 0, 0, 0 };
	int angle_pwm = 0;
	int angle = 0;
	int angle_rpm   = 0;
	int speed = 0;

	int voltage = 0;
	int direction = 0;
	int init_state = INIT;
	int pwm_half = PWM_MAX_VALUE>>1;
	//int angle_variance = commutation_params.angle_variance;
	//int max_speed_reached = commutation_params.max_speed_reached;

int max_count_per_hall = qei_params.real_counts/hall_params.pole_pairs;

	while (1)
	{
		//xscope_probe_data(0, direction);

//		if(sensor_select == 1)
		{
			speed = get_hall_velocity(c_qei, hall_params);//get_hall_velocity(c_hall, hall_params);// get_hall_velocity(c_hall, hall_params);
		//	angle = get_hall_position(c_hall);
			angle = (get_sync_position(c_sync) << 12)/max_count_per_hall;
		}


//		angle_rpm = speed;
//		if(angle_rpm < 0)
//			angle_rpm = -angle_rpm;

		angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;
//		angle_rpm *= 28;		// fixed variation range
//		angle_rpm /= 3000; 		//  should be settable (equal to max_rpm)


		if(voltage<0)
			direction = -1;
		else if(voltage >= 0)
			direction = 1;


		if (direction == 1)
		{
			angle_pwm = ((angle +  450) & 0x0fff) >> 2;					//100 M3  //100 M1 //180           old 480
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;
			pwm[1] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_reduce(angle_pwm))*voltage)/13889   + pwm_half;

		}
		else if (direction == -1)
		{
			angle_pwm = ((angle  + 3100 ) & 0x0fff) >> 2;  				//2700 M3  //  2550 M1 //2700      old 3000
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm +341) & 0x3ff;

			pwm[1] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;
			angle_pwm = (angle_pwm + 342) & 0x3ff;
			pwm[2] = ((sine_reduce(angle_pwm))*-voltage)/13889   + pwm_half;

		}

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

		select
		{
			case c_commutation_p1 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p1 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p1 :> commutation_params.angle_variance;
					c_commutation_p1 :> commutation_params.max_speed_reached;
				}
				break;
			case c_commutation_p2 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p2 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p2 :> commutation_params.angle_variance;
					c_commutation_p2 :> commutation_params.max_speed_reached;
				}
				break;
			case c_commutation_p3 :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation_p3 :> voltage;
				}
				else if(command == SET_COMMUTATION_PARAMS)
				{
					c_commutation_p3 :> commutation_params.angle_variance;
					c_commutation_p3 :> commutation_params.max_speed_reached;
				}
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
}
/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage)
{
	c_commutation <: 2;
	c_commutation <: input_voltage;
	return;
}

void commutation_sinusoidal(int sensor_select, hall_par &hall_params, qei_par &qei_params, commutation_par &commutation_params, chanend c_hall, chanend c_qei, chanend c_pwm_ctrl, chanend signal_adc,
		chanend c_signal, chanend c_sync, chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3)
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
	  while(1)
	  {
		  unsigned command, received_command = 0;
		  #pragma ordered
		  select
		  {
			case signal_adc :> command:
				received_command = 1;
				//printstrln("received signal from torque ctrl");
				signal_adc <: 1;
				break;
			case t when timerafter(ts + timeout) :> void:
				received_command = 1;
				//printstrln("timed out");
				break;
			case c_signal :> command:  //
				if(command == CHECK_BUSY)
					c_signal <: init_state;
				break;
			default:
				break;
		  }
		  if(received_command == 1)
			  break;
	  }

	 // printstrln("start commutation");

	  if( sensor_select ==  HALL)
		  commutation_sinusoidal_loop(sensor_select, hall_params, qei_params, commutation_params, c_hall, c_qei, c_pwm_ctrl, c_signal, c_commutation_p1, c_commutation_p2, c_commutation_p3);
	  else if(sensor_select == QEI)
		  commutation_sinusoidal_loop_qei( qei_params,hall_params, commutation_params, c_hall, c_pwm_ctrl, c_signal, c_sync, c_commutation_p1, c_commutation_p2, c_commutation_p3);
}





