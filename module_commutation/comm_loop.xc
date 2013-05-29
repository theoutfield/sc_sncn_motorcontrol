
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

static t_pwm_control pwm_ctrl;

void commutation_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}


/* FOC based commutation functions */
void space_vector_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff );

void commutation_loop(chanend c_value, chanend c_pwm_ctrl)  //advanced for foc based commutation
{
	int cmd, iPwmOnOff = 1;
	int iIndexPWM=0, iUmotMotor = 0, iMotHoldingTorque = 0;

	 //================== pwmloop ========================
	while (1)
	{
		select
		{
			case c_value :> cmd:     		// loop or user cmd token 40
				if(cmd == 40)
				{
					c_value :> cmd;   		// from user or other external loops
					iUmotMotor = cmd;      	// also can be done but regulated angle
					c_value :> cmd;
					iIndexPWM = cmd;
				}
				break;
			default:
				break;

		}

		space_vector_pwm( iIndexPWM, iUmotMotor, iMotHoldingTorque, pwm_ctrl, c_pwm_ctrl, iPwmOnOff );

	}

}

void commutation(chanend c_value, chanend c_pwm_ctrl, chanend sig)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  commutation_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  sig <: 1;
	  while(1)
	  {
		  unsigned cmd, found =0;
		  select
		  {
			case sig :> cmd:
				found = 1;
				break;
			default:
				break;
		  }
		  if(found == 1)
			  break;
	  }
	  commutation_loop(c_value, c_pwm_ctrl);
}


/* Sinusoidal based commutation functions */

void commutation_sinusoidal_loop( chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl)
{
	unsigned cmd;
	unsigned pwm[3] = { 0, 0, 0 };
	int iIndexPWM, iPosFromHallOld=0;
	int iAngleFromHall  = 0;
	int iAngleUser      = 300;
	int iAngleFromRpm   = 0;
	int iAnglePWM, iActualSpeed = 0;
	int i;

	int umot =0, umot1 = 0, dir = 0, speed = 0, stop = 0, set = 0,limit=20;


	//t:>time;
	while (1)
	{
		//============= rotor position ===============================
		if(stop!=1)
		{
			iActualSpeed = get_speed_cal(c_hall);
			iAngleFromHall = get_hall_angle(c_hall);
		}
		else
		{
			iActualSpeed = get_speed_cal(c_hall);
			iAngleFromHall = 30;
			umot1 = 2000;
			if(set>limit)
			{
				set=limit;
			}
			umot1=set*100;

		}


		iAngleFromRpm = iActualSpeed;
		if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;
		iAngleFromRpm *= 150;
		iAngleFromRpm /= 4000;


		if(umot1<0)
			dir = -1;
		if(umot1 >= 0)
			dir = 1;


		if (dir == 1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser + iAngleFromRpm  + 180;//100 M3  //100 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*umot1)/13889  + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*umot1)/13889   + PWM_MAX_VALUE/2;

		}

		if (dir == -1)
		{
			iAnglePWM = iAngleFromHall + iAngleUser - iAngleFromRpm + 2700;  //2700 M3  //  2550 M1
			iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
			iIndexPWM = iAnglePWM >> 4;
			pwm[0] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM +85) & 0xff;
			pwm[1] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;
			iIndexPWM = (iIndexPWM + 86) & 0xff;
			pwm[2] = ((sine_third[iIndexPWM])*-umot1)/13889   + PWM_MAX_VALUE/2;

		}

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

		select {
			case c_commutation :> cmd:
			  if(cmd==3)
			  {
				  stop=1;
				  c_commutation :> set;
			  }
			  else if(cmd==2){ 	// set Umot
				  stop=0;
				  c_commutation :> umot1;
			  }
			  else if(cmd == 4) // set Direction
			  {
				  stop=0;
				  c_commutation :> dir;
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

void commutation_sinusoidal(chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend signal_adc)
{
	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  commutation_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	 /* signal_adc <: 1;
	  while(1)
	  {
		  unsigned cmd, found =0;
		  select
		  {
			case signal_adc :> cmd:
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





