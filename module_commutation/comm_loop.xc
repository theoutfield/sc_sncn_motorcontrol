
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
#include "comm_loop.h"
#include <internal_config.h>
#include "print.h"

#define SET_VOLTAGE    2

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

/* Sinusoidal based commutation functions */

void commutation_sinusoidal_loop( chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl)
{
	unsigned command;
	unsigned pwm[3] = { 0, 0, 0 };
	int angle_pwm;
	int angle;
	int angle_rpm   = 0;
	int speed = 0;

	int voltage = 0;
	int direction = 0;

	unsigned time;
	timer t;

	t :> time;
	while (1)
	{

		speed = get_speed_cal(c_hall);
		angle = get_hall_angle(c_hall);


		angle_rpm = speed;
		if(angle_rpm < 0)
			angle_rpm = -angle_rpm;

		angle_rpm *= 150;		// fixed variation range
		angle_rpm /= 6780; 		// TODO should be settable (equal to max_rpm)


		if(voltage<0)
			direction = -1;
		else if(voltage >= 0)
			direction = 1;


		if (direction == 1)
		{
			angle_pwm = ((angle + angle_rpm  + 480) & 0x0fff) >> 4;					//100 M3  //100 M1 //180         TODO parameter
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_third[angle_pwm])*voltage)/13889   + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm +85) & 0xff;
			pwm[1] = ((sine_third[angle_pwm])*voltage)/13889   + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm + 86) & 0xff;
			pwm[2] = ((sine_third[angle_pwm])*voltage)/13889   + PWM_MAX_VALUE/2;

		}
		else if (direction == -1)
		{
			angle_pwm = ((angle - angle_rpm + 3000) & 0x0fff) >> 4;  				//2700 M3  //  2550 M1 //2700     TODO parameter
																					// 0 - 4095  -> 0x0000 - 0x0fff
			pwm[0] = ((sine_third[angle_pwm])*-voltage)/13889   + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm +85) & 0xff;
			pwm[1] = ((sine_third[angle_pwm])*-voltage)/13889   + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm + 86) & 0xff;
			pwm[2] = ((sine_third[angle_pwm])*-voltage)/13889   + PWM_MAX_VALUE/2;

		}

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

		select {
			case c_commutation :> command:
				if(command == SET_VOLTAGE)				// set voltage
				{
					c_commutation :> voltage;
				}
				break;

			default:
				break;
		}

		t when timerafter( time + 13889 ) :> time;
	}

}


/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage)
{
	c_commutation <: 2;
	c_commutation <: input_voltage;
	return;
}

void commutation_sinusoidal(chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend signal_adc, chanend c_signal)
{
	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  commutation_init_to_zero(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  printstrln("start");
	  c_signal <: 1; 			//signal commutation init done.

	 /* signal_adc <: 1;
	  while(1)
	  {
		  unsigned command, received_command = 0;
		  select
		  {
			case signal_adc :> command:
				received_command = 1;
				break;
			default:
				break;
		  }
		  if(received_command == 1)
			  break;
	  }
*/
	  commutation_sinusoidal_loop( c_commutation, c_hall, c_pwm_ctrl);

}





