/*
 *
 * \Motor Control file
 * \brief Main project file
 *
 * Port declarations, etc.
 * 
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
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
#include "xmos_pm.h"
#include "comm_loop.h"
#include "adc_ad7949.h"
#include "set_cmd.h"
#include "hall_input.h"


#define COM_CORE 0
#define IFM_CORE 3

//on stdcore[IFM_CORE]: clock clk_uart = XS1_CLKBLK_REF;
on stdcore[IFM_CORE]: clock clk_adc  = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm  = XS1_CLKBLK_REF;


int main(void)
{
  chan c_adc;
  chan c_adctrig;
  chan c_hall;
  chan c_pwm_ctrl;
  chan c_commutation;
  chan c_motvalue;

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
/*
			xscope_register(7,
			XSCOPE_CONTINUOUS, "0 a1", XSCOPE_INT, "n",
			XSCOPE_CONTINUOUS, "1 a2", XSCOPE_INT, "n",
			XSCOPE_CONTINUOUS, "2 iSetLoopSpeed", XSCOPE_INT, "n",
			XSCOPE_CONTINUOUS, "3 iAngleFromHall", XSCOPE_INT, "n",
			XSCOPE_CONTINUOUS, "4 iAngleCur", XSCOPE_INT, "n",
			XSCOPE_CONTINUOUS, "5 iUmotMotor", XSCOPE_UINT, "n",
			XSCOPE_CONTINUOUS, "6 iIq", XSCOPE_UINT, "n"
			);


			 xscope_register(7,
			 XSCOPE_CONTINUOUS, "0 a1RMS", XSCOPE_INT, "n",
			 XSCOPE_CONTINUOUS, "1 iActualSpeed", XSCOPE_INT, "n",
			 XSCOPE_CONTINUOUS, "2 iSetLoopSpeed", XSCOPE_INT, "n",
			 XSCOPE_CONTINUOUS, "3 iUmotIntegrator", XSCOPE_INT, "n",
			 XSCOPE_CONTINUOUS, "4 iUmotMotor", XSCOPE_INT, "n",
			 XSCOPE_CONTINUOUS, "5 iAngleDiffPeriod", XSCOPE_UINT, "n",
			 XSCOPE_CONTINUOUS, "6 iIqPeriod2", XSCOPE_UINT, "n"
			);
 */


	 	 xscope_register(10,
		 XSCOPE_CONTINUOUS, "0 iPhase1", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "1 iAngleCurrent", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "2 iAnglePWM", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "3 iAngleFromHall",XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "4 iAngleInvPark", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "5 iAnglePWMFromHall", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "6 iAnglePWMFromFOC", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "7 iVectorCurrent", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "8 iVectorInvPark", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "9 iPhase2", XSCOPE_INT, "n"
		);
    }
    /************************************************************
     * CORE 2             communication with the Motor
     ************************************************************/

     on stdcore[2]:par{
    	{
			  cmd_data send_cmd;
			  int valid = 0;
			  timer tx;
			  unsigned ts;
              int iIndex1;

              iIndex1=0; while(iIndex1 < 32)send_cmd.iMotValues[iIndex1++] =0;
              iIndex1=0; while(iIndex1 < 32)send_cmd.iMotPar[iIndex1++]	=0;
              iIndex1=0; while(iIndex1 < 16)send_cmd.iMotCommand[iIndex1++]=0;
              send_cmd.varx=0;
              send_cmd.var1=0;

              tx :> ts;  // first value
			  tx when timerafter(ts+1*SEC_FAST) :> ts;


			  while(1)
			  {
				  valid = input_cmd(send_cmd);   // valid command entered
				  if(valid == 1)				 // if valid send command to motor ( cmd from 0 to 31 )
				  {
					  send_cmd.iMotCommand[15]=1;
					  iIndex1=0;
					  while(iIndex1 <= 12)
					  {
		 	    	  c_commutation <:iIndex1;
		 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1];
		 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+1];
		 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+2];
		 	    	  c_commutation <: send_cmd.iMotCommand[iIndex1+3];
		 	  	  	  iIndex1 += 4;
					  }
				  }// end if valid == 1


				  if(send_cmd.varx == 1)  // readout motor values (cmd from 32 to 63 )
				  {
					  iIndex1 = 32;
					  while(iIndex1 < 64)
					  {
					  c_commutation <: iIndex1; 	c_commutation :> send_cmd.iMotValues[iIndex1-32];
					  iIndex1++;
					  }
				  }

				  if(valid >= 96 && valid < 128)  // send actual parameter
				  {  c_commutation <: valid;	  c_commutation <: send_cmd.var1;  }

				  if(send_cmd.varx == 2)  	// readout  parameters (cmd from 64 to 97 )
				  {
					  iIndex1 =64;
					  while(iIndex1 < 96) {
						  c_commutation <: iIndex1; 	c_commutation :> send_cmd.iMotPar[iIndex1-64];
						  iIndex1++;
					  }
				  }

			  }//end while 1
    	}
    }// end on stdcore[2]
    
    /************************************************************
     * IFM_CORE
     ************************************************************/
    on stdcore[IFM_CORE]: {
    	par {
    			adc_ad7949_triggered( c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

    			//do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, ADC_SYNC_PORT, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);
    			do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

    			run_hall( c_hall, p_ifm_hall, p_ifm_encoder);
    			commutation(c_adc, c_commutation, c_hall, c_pwm_ctrl, c_motvalue );

    			run_uart(c_motvalue, clk_pwm);

      	  }
    }// end stdcore[IFM_CORE]

  }// end par main
  return 0;
}


