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
	//	#ifdef USE_XSCOPE
    	/*
			  	  	 xscope_register(7,
					 XSCOPE_CONTINUOUS, "0 iPhase1", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "1 iPhase2", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "2 uCurVector", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "3 iAngleCur", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "4 position1", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "5 iIdfiltered", XSCOPE_UINT, "n",
					 XSCOPE_CONTINUOUS, "6 iIqfiltered", XSCOPE_UINT, "n"
					);
		*/
 	  	 	 	 	 xscope_register(5,
					 XSCOPE_CONTINUOUS, "0 ia", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "1 ib", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "2 ic", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "3 iAngleHall", XSCOPE_INT, "n",
					 XSCOPE_CONTINUOUS, "4 iAngleCur", XSCOPE_INT, "n"
					// XSCOPE_CONTINUOUS, "5 iIdfiltered", XSCOPE_UINT, "n",
					// XSCOPE_CONTINUOUS, "6 iIqfiltered", XSCOPE_UINT, "n"
					);

		//			 XSCOPE_CONTINUOUS, "2 iRampAccValue", XSCOPE_INT, "n",
		//			 XSCOPE_CONTINUOUS, "3 iRampDecValue", XSCOPE_INT, "n",
			  xscope_config_io(XSCOPE_IO_BASIC);
//		#endif}
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
              int iVarArray[24];

              send_cmd.var1=0;
              send_cmd.var2=0;
              send_cmd.var3=0;
              send_cmd.var4=0;
              send_cmd.var5=0;
              send_cmd.var6=0;
              send_cmd.var7=0;
              send_cmd.var8=0;
              send_cmd.var9=0;
              send_cmd.var10=0;
              send_cmd.var11=0;
              send_cmd.var12=0;

			  t when timerafter(time+1*SEC_FAST) :> time;

			  while(1)
			  {
				  valid = input_cmd(send_cmd);   // valid command entered

				  if(send_cmd.varx == 1)
				  {
				    c_commutation <: 8;  	c_commutation :> iVarArray[8]; 		send_cmd.var13  = iVarArray[8];
				    c_commutation <: 9;  	c_commutation :> iVarArray[9]; 		send_cmd.var14  = iVarArray[9];
				    c_commutation <: 10;  	c_commutation :> iVarArray[10]; 	send_cmd.var15  = iVarArray[10];

				  	c_commutation <: 11;  	c_commutation :> iVarArray[11]; 	send_cmd.var1  = iVarArray[11];
				  	c_commutation <: 12;    c_commutation :> iVarArray[12]; 	send_cmd.var2  = iVarArray[12];
				  	c_commutation <: 13;    c_commutation :> iVarArray[13]; 	send_cmd.var3  = iVarArray[13];
				  	c_commutation <: 14;    c_commutation :> iVarArray[14]; 	send_cmd.var4  = iVarArray[14];
				  	c_commutation <: 15;    c_commutation :> iVarArray[15]; 	send_cmd.var5  = iVarArray[15];
				  	c_commutation <: 16;    c_commutation :> iVarArray[16]; 	send_cmd.var6  = iVarArray[16];
				  	c_commutation <: 17;    c_commutation :> iVarArray[17]; 	send_cmd.var7  = iVarArray[17];
				  	c_commutation <: 18;    c_commutation :> iVarArray[18]; 	send_cmd.var8  = iVarArray[18];
				  	c_commutation <: 19;    c_commutation :> iVarArray[19]; 	send_cmd.var9  = iVarArray[19];
				  	c_commutation <: 20;    c_commutation :> iVarArray[20]; 	send_cmd.var10 = iVarArray[20];
				  	c_commutation <: 21;    c_commutation :> iVarArray[21]; 	send_cmd.var11 = iVarArray[21];
				  	c_commutation <: 22;    c_commutation :> iVarArray[22]; 	send_cmd.var12 = iVarArray[22];
				  }

				  if(valid == 1)				// if valid send command to motor
				  {
					  c_commutation <:1;
					  c_commutation <:send_cmd.iPwmOnOff;	//ON OFF PWM

					  c_commutation <:4;
					  c_commutation <:send_cmd.iSetValueSpeedRpm;	//Speed RPM

					  c_commutation <:5;
					  c_commutation <:send_cmd.iAngleVariable;		//Angle
				  }// end valid
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

    			run_hall( c_hall, p_ifm_hall);
    			commutation(c_adc, c_commutation, c_hall, c_pwm_ctrl );

    			{

    				// c_commutation <:1;
    				// c_commutation <:1;	//ON OFF PWM

					// c_commutation <:4;
					 //c_commutation <:4000;	//Speed RPM
    			}
      	  }
    }// end stdcore[IFM_CORE]

  }// end par main
  return 0;
}


