
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

void comm_sine(chanend c_value, chanend c_pwm_ctrl)
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

	}// end while(1)

}// end function





void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



void commutation(chanend c_value, chanend c_pwm_ctrl, chanend sig)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
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
	  comm_sine(c_value, c_pwm_ctrl);
}
