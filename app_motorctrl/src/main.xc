/**
 * \file main.xc
 *	Motor Control mainfile
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com> ,Ludwig Orgler <orgler@tin.it> & Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/
#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <xscope.h>
#include "ioports.h"
#include "refclk.h"
#include "pwm_service_inv.h"
#include "xmos_pm.h"
#include "flash_Somanet.h"
#include "comm_loop.h"
#include "adc_ad7949.h"
#include "set_cmd.h"
#include "hall_input.h"


#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc  = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm  = XS1_CLKBLK_REF;


int main(void)
{
  chan c_adc;
  chan c_adctrig;
  chan c_hall;
  chan c_pwm_ctrl;
  chan c_commutation;
  chan dummy, c_hall_1, input, c_value, sig;

  par
  {
		/*************************************************************
	     * COM_CORE
	     *************************************************************/
	    on stdcore[COM_CORE]: {
	      enableAEC(500);
	    }

	    /************************************************************
	     * CORE 1
	     ************************************************************/
	    on stdcore[1]: {
				 xscope_register(9,
				 XSCOPE_CONTINUOUS, "1 iActualSpeed", XSCOPE_INT, "n",
				 XSCOPE_CONTINUOUS, "2 iSetLoopSpeed", XSCOPE_INT, "n",
				 XSCOPE_CONTINUOUS, "3 iUmotIntegrator", XSCOPE_INT, "n",
				 XSCOPE_CONTINUOUS, "4 iUmotMotor", XSCOPE_INT, "n",
				 XSCOPE_CONTINUOUS, "5 iAngleDiffPeriod", XSCOPE_UINT, "n",
				 XSCOPE_CONTINUOUS, "6 iVectorInvPark", XSCOPE_UINT, "n",
				 XSCOPE_CONTINUOUS, "7 iVectorCurrent", XSCOPE_UINT, "n",
				 XSCOPE_CONTINUOUS, "8 iIdPeriod2", XSCOPE_UINT, "n",
				 XSCOPE_CONTINUOUS, "9 iIqPeriod2", XSCOPE_UINT, "n"
				);

	    }

    /************************************************************
     * CORE 2             communication with the Motor
     ************************************************************/
    on stdcore[2]:par{
        	{
        		tor_data send;
        		timer tx;unsigned  ts;int valid=0;
        		int torq;
        		 tx :> ts;  // first value
        					  tx when timerafter(ts+4*SEC_FAST) :> ts;
        		while(1)
        		{
        			valid = input_tor_cmd(send );
        			printintln( send.var1);
        			torq = send.var1;
        			input <: 20;
        			input <: torq;
        		}
        	}
    }

    
    /************************************************************
     * IFM_CORE
     ************************************************************/
    on stdcore[IFM_CORE]: {
    	par {
    			adc_ad7949_triggered( c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

    			do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

    			run_hall( c_hall, p_ifm_hall, p_ifm_encoder);

    			commutation(c_commutation, c_value, c_pwm_ctrl, sig);

    			foc_loop(sig, input, c_adc, c_hall, c_value);
      	  }
    }// end stdcore[IFM_CORE]

  }// end par main
  return 0;
}



