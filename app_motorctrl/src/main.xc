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
#include "comm_sine.h"
#include "adc_ad7949.h"
#include "set_cmd.h"
#include "hall_input.h"


#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


int main(void)
{
  chan c_adc;
  chan c_adctrig;
  chan c_hall;
  chan c_pwm_ctrl;
  chan c_commutation;


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


	 	 xscope_register(11,
		 XSCOPE_CONTINUOUS, "0 iPhase1", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "1 iAngleCurrent", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "2 iAnglePWM", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "3 iAngleFromHall",XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "4 iAngleInvPark", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "5 iAngleDiffFOC", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "6 iAnglePWMFromHall", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "7 iAnglePWMFromFOC", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "8 iVectorInvPark",XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "9 iDiffAngleHall", XSCOPE_INT, "n",
		 XSCOPE_CONTINUOUS, "10 iAngleXXX",XSCOPE_INT, "n"
		);


    }
    /************************************************************
     * CORE 2             communication with the Motor
     ************************************************************/

    on stdcore[2]:par{
    	{
			  cmd_data send_cmd;
			  int valid = 0;
			  timer t;
			  unsigned time;
              int iIndex;

              send_cmd.varx=0;
              send_cmd.var1=0;


			  t when timerafter(time+1*SEC_FAST) :> time;

			  while(1)
			  {
				  valid = input_cmd(send_cmd);   // valid command entered

				  if(valid == 1)				// if valid send command to motor ( cmd from 0 to 31 )
				  {
					  c_commutation <:1;	  c_commutation <:send_cmd.iPwmOnOff;			//ON OFF PWM
					  c_commutation <:2;	  c_commutation <:send_cmd.iHoldingTorque;
					  c_commutation <:3;	  c_commutation <:send_cmd.iTorqueSetValue;
					  c_commutation <:4;	  c_commutation <:send_cmd.iSetValueSpeedRpm;	//Speed RPM
					  c_commutation <:5;	  c_commutation <:send_cmd.iControlFOCcmd;
				  }


				  if(send_cmd.varx == 1)  // readout motor values (cmd from 32 to 63 )
				  {
					  iIndex = 32;
					  while(iIndex < 64)
					  {
					  c_commutation <: iIndex; 	c_commutation :> send_cmd.iMotValues[iIndex-32];
					  iIndex++;
					  }
				  }

				  if(send_cmd.varx == 2)  	// readout  parameters (cmd from 64 to 97 )
				  {
					  iIndex =64;
					  while(iIndex < 96) {
						  c_commutation <: iIndex; 	c_commutation :> send_cmd.iMotPar[iIndex-64];
						  iIndex++;
					  }
				  }

				  if(valid >= 96 && valid < 128)  // send actual parameter
				  {  c_commutation <: valid;	  c_commutation <: send_cmd.var1;  }


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
    			commutation(c_adc, c_commutation, c_hall, c_pwm_ctrl );
      	  }
    }// end stdcore[IFM_CORE]

  }// end par main
  return 0;
}


