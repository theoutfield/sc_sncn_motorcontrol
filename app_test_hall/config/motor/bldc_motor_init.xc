
/**
 * \file bldc_motor_init.xc
 * \brief Motor Control config initialization functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
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
