/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *\Motor 3 motion profile size optimized code for position ctrl loops
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include "ioports.h"
#include "refclk.h"
#include "watchdog.h"




//#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;



int main(void) {
	//watchdog
	chan c_wd;


	par
	{



		on stdcore[1]:
		{
			timer t;
			unsigned time;

			// enable watchdog
			//	printstrln("send command to watchdog");

			t :> time;
			t when timerafter (time+250000*4):> time;
			//	wd_testbench (c_wd);
			c_wd <: WD_CMD_START;

			//printstrln("command sent");

		}



		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{
				do_wd(c_wd,  p_ifm_wd_tick,  p_ifm_shared_leds_wden);
			}
		}

	}

	return 0;
}
