#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"
#include <bldc_motor_config.h>


void qei_unit_test()
{
	int max, min;
	qei_par qei_params;
	in_data d;
	while(1)
	{
		input_max_position(d);
		input_min_position(d);
//		init_qei_param(qei_params, d.max_position, d.min_position);
		printstr("\n Max ticks: ");printintln(qei_params.max_ticks);
	}

}

