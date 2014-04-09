
/**
 * \file watchdog.xc
 * \brief Watchdog Implementation
 * \author Victor de la Cruz <vdelacruz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH & XMOS Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */


#include "watchdog.h"
#include <xs1.h>


void run_watchdog(chanend c_watchdog, out port p_wd_tick, out port p_shared_leds_wden)
{
	unsigned int cmd, wd_enabled = 1, shared_out = 0xe, tick_out = 0;
	unsigned int ts, ts2;
	timer t;

	t :> ts;

	// Loop forever processing commands
	while (1)
	{
		select
		{
			// Get a command from the out loop
			case c_watchdog :> cmd:
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
