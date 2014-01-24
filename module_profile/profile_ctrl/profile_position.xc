
/**
 *
 * \file profile_position.xc
 *
 * \brief Profile Position Control functions
 * 	Implements position profile control function
 *
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
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


#include "position_ctrl_client.h"
#include "refclk.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include <profile.h>
#include <profile_control.h>

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration, \
		int max_position, int min_position, chanend c_position_ctrl)
{
	int i;
	timer t;
	unsigned int time;
	int steps;
	int position_ramp;

	int actual_position = 0;

	int init_state = __check_position_init(c_position_ctrl);
	if(init_state == INIT_BUSY)
	{
		init_state = init_position_control(c_position_ctrl);
	}

	if(init_state == INIT)
	{
		target_position = position_limit(target_position, max_position, min_position);
		actual_position = get_position(c_position_ctrl); //degree * 10000
		steps = init_position_profile(target_position*10000, actual_position, velocity, acceleration, deceleration);
		t :> time;
		for(i = 1; i < steps; i++)
		{
			position_ramp = position_profile_generate(i);
			set_position(position_ramp, c_position_ctrl);
			actual_position = get_position(c_position_ctrl);

			t when timerafter(time + MSEC_STD) :> time;
			/*xscope_probe_data(0, actual_position);
			xscope_probe_data(1, position_ramp);*/
		}
		t when timerafter(time + 30 * MSEC_STD) :> time;
	}
}
