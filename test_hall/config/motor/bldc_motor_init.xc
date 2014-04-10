
/**
 * \file bldc_motor_init.xc
 * \brief Motor Control config initialization functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
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

#include <bldc_motor_config.h>
#include "refclk.h"

void init_hall_param(hall_par &hall_params)
{
	int max = MAX_POSITION_LIMIT;
	int min = MIN_POSITION_LIMIT;
	hall_params.pole_pairs = POLE_PAIRS;

	if(max >= 0 && min >= 0)
	{
		if(max > min)
			hall_params.max_ticks = max;
		else
			hall_params.max_ticks = min;
	}
	else if(max <= 0 && min <= 0)
	{
		if(max < min)
			hall_params.max_ticks = -max;
		else
			hall_params.max_ticks = -min;
	}
	else if(max > 0 && min < 0)
	{
		if(max > 0 - min)
			hall_params.max_ticks = max;
		else
			hall_params.max_ticks = 0 - min;
	}
	else if(max < 0 && min > 0)
	{
		if(min > 0 - max)
			hall_params.max_ticks = min;
		else
			hall_params.max_ticks = 0 - max;
	}
	hall_params.max_ticks_per_turn = POLE_PAIRS * 4096;
	//printintln(hall_params.max_ticks);
	hall_params.max_ticks += hall_params.max_ticks_per_turn ;  // tolerance
	//printintln(hall_params.max_ticks);

	return;
}
