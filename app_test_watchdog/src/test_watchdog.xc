
/**
 * \file test_watchdog.xc
 * \brief Watchdog test
 * \author  Martin Schwarz <mschwarz@synapticon.com>
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <refclk.h>
#include <watchdog.h>


#define COM_CORE 0
#define IFM_CORE 3

on tile[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on tile[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;


int main(void)
{

	chan c_watchdog; 		// watchdog channel

	par
	{

		on tile[1]:
		{
			timer t;
			unsigned int time;

			// enable watchdog thread
			t :> time;
			t when timerafter (time+250000*4):> time;
			c_watchdog <: WD_CMD_START;
		}


		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on tile[IFM_CORE]:
		{
			run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);
		}

	}

	return 0;
}
