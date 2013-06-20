
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
#include "print.h"

static t_pwm_control pwm_ctrl;

void commutation_init(chanend c_pwm_ctrl)
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

	int voltage = 0, direction = 0, stop = 0, set = 0, limit=20;


	//t:>time;
	while (1)
	{
		//============= rotor position ===============================
		if(stop!=1)
		{
			speed = get_speed_cal(c_hall);
			angle = get_hall_angle(c_hall);
		}
		else
		{
			speed = get_speed_cal(c_hall);
			angle = 30;
			voltage = 2000;
			if(set>limit)
			{
				set=limit;
			}
			voltage=set*100;

		}


		angle_rpm = speed;
		if(angle_rpm < 0)
			angle_rpm = -angle_rpm;

		angle_rpm *= 150;	// fixed variation
		angle_rpm /= 6780; // settable should equal to max_rpm


		if(voltage<0)
			direction = -1;
		if(voltage >= 0)
			direction = 1;


		if (direction == 1)
		{
			angle_pwm = angle + angle_rpm  + 480;//100 M3  //100 M1 //180
			angle_pwm &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			angle_pwm = angle_pwm >> 4;
			pwm[0] = ((sine_third[angle_pwm])*voltage)/13889  + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm +85) & 0xff;
			pwm[1] = ((sine_third[angle_pwm])*voltage)/13889   + PWM_MAX_VALUE/2;
			angle_pwm = (angle_pwm + 86) & 0xff;
			pwm[2] = ((sine_third[angle_pwm])*voltage)/13889   + PWM_MAX_VALUE/2;

		}

		if (direction == -1)
		{
			angle_pwm = angle - angle_rpm + 3000;  //2700 M3  //  2550 M1 //2700
			angle_pwm &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			angle_pwm = angle_pwm >> 4;
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
			  if(command==3)
			  {
				  stop=1;
				  c_commutation :> set;
			  }
			  else if(command==2){ 	// set Umot
				  stop=0;
				  c_commutation :> voltage;
			  }
			  else if(command == 4) // set Direction
			  {
				  stop=0;
				  c_commutation :> direction;
			  }
			  break;
			default:
			  break;
		}

	}

}


/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input)
{
	c_commutation <: 2;
	c_commutation <: input;
	return;
}

void commutation_sinusoidal(chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend signal_adc, chanend c_signal)
{
	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  commutation_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  printstrln("start");
	  c_signal <: 1; //driver init done.
	 /* signal_adc <: 1;
	  while(1)
	  {
		  unsigned command, found =0;
		  select
		  {
			case signal_adc :> command:
				found = 1;
				break;
			default:
				break;
		  }
		  if(found == 1)
			  break;
	  }
*/
	  commutation_sinusoidal_loop( c_commutation, c_hall, c_pwm_ctrl);

}





