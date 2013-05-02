/*
 *
 * \Motor Control file
 * \brief Main project file
 *
 * Port declarations, etc.
 * 
 * \author  Ludwig Orgler orgler@tin.it  Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.2 (2013-05-02)
 * \
*/

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include <xscope.h>
#include "somanet/ioports.h"
#include "refclk.h"
#include "pwm_service_inv.h"

#include "comm_loop.h"
#include "adc_ad7949.h"
#include "set_cmd.h"
#include "hall_server.h"


#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc  = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm  = XS1_CLKBLK_REF;

void run_console(chanend c_commutation);

int main(void)
{
	/***********************Motor Control channels*************************/
	chan c_adc; 						///< ADC Channels
	chan c_hall;						///< Hall position Channels
	chan c_encoder;						///< encoder channel
	chan c_pwm_ctrl; 					///< Commutation Channnels
	chan c_commutation;

	/**********************************************************************
	 * CORE 0            	Ethercat communication
	 **********************************************************************/
	par{

			on stdcore[0] : {
		//		enableAEC(500);

			}

    /************************************************************
     * CORE 1
     ************************************************************/
    on stdcore[1]: {

    }
    /************************************************************
     * CORE 2             communication with the Motor
     ************************************************************/

    on stdcore[2]:{
    	par	{

    		run_console(c_commutation);
    	}
    }// end on stdcore[2]



    /************************************************************
     * IFM_CORE
     ************************************************************/
    on stdcore[IFM_CORE]: {
    	par {
    			adc_ad7949_triggered( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

    			do_pwm_inv(c_pwm_ctrl, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

#ifdef defHALL
    			run_hall( c_hall, p_ifm_hall );
#endif

#ifdef defENCODER
    			run_encoder( c_encoder, p_ifm_encoder);
#endif

    			commutation(c_adc, c_commutation, c_hall, c_encoder, c_pwm_ctrl  );

      	  }
    }// end stdcore[IFM_CORE]

  }// end par main
  return 0;
}


//===========================================================================================================================
//
//===========================================================================================================================

void run_console(chanend c_commutation)
{
	cmd_data send_cmd;
	int valid = 0;
	timer tx;
	unsigned ts;
	int iIndex1;

	iIndex1=0; while(iIndex1 < 16)send_cmd.iMotCommand[iIndex1++]=0;
	            send_cmd.iMotCommand[1]=2;
	            send_cmd.varx=0;
	            send_cmd.var1=0;

				  iIndex1=0; while(iIndex1 < 16)send_cmd.iMotCommand[iIndex1++]=0;
	              send_cmd.iMotCommand[1]=2;
	              send_cmd.varx=0;
	              send_cmd.var1=0;

	              tx :> ts;  // first value
				  tx when timerafter(ts+1*SEC_FAST) :> ts;

				  while(1)
				  {
					  valid = input_cmd(send_cmd);   // valid command entered
					  if(valid == 1)				 // if valid send command to motor ( cmd from 0 to 31 )
					  {
						  send_cmd.iMotCommand[7]=1;
						  iIndex1=0;
						  while(iIndex1 < 7)
						  {
			 	    	  c_commutation <:iIndex1;
			 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1];
			 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+1];
			 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+2];
			 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+3];
			 	  	  	  iIndex1 += 4;
						  }
					  }// end if valid == 1
				  }//end while 1
}

