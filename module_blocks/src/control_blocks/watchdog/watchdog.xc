/**
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Victor de la Cruz <vdelacruz@synapticon.com>
 * Version: 0.1 (2013-08-22 1505)
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 *
 */


#include "watchdog.h"
#include <xs1.h>


void do_wd(chanend c_wd, out port p_wd_tick, out port p_shared_leds_wden)
{
	unsigned cmd, wd_enabled = 1, shared_out = 0xe, tick_out = 0;
	unsigned ts, ts2;
	timer t;

	t :> ts;

	// Loop forever processing commands
	while (1)
	{
		select
		{
			// Get a command from the out loop
			case c_wd :> cmd:
				switch(cmd)
				{
					case WD_CMD_START: // produce a rising edge on the WD_EN
						shared_out &= ~0x1;
						p_shared_leds_wden <: shared_out; // go low
						t :> ts2;
						t when timerafter(ts2+25000) :> ts2;
						shared_out |= 0x1;
						p_shared_leds_wden <: shared_out; // go high
						break;
	
					// if the watchdog is enabled, kick it
					case WD_CMD_DIS_MOTOR :
						// mark that the watchdog should now not run
						wd_enabled = 0;
						break;
				}
				break;
				
			case t when timerafter(ts + 250000) :> ts:
				if ( wd_enabled == 1 )
				{
					tick_out ^= 0x1;
				}
				
				break;


			// Do a case: Separate the LED bits from the port XS1_PORT_4B. LSB enable watchdog.
		}
		
		// Send out the new value to the shared port
		p_shared_leds_wden <: shared_out;
		p_wd_tick <: tick_out;
	}
}
