/*
#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"
#include "comm_sine.h"
#include "hall_client.h"

void set_position_test(chanend c_position_ctrl)
{
	int position = 0;
	in_data d;
	timer ts;
	unsigned time;
	int increment = 50;

	ts:>time;


	while (1) {
		//input_pos(d);
		//printintln(d.set_position);

		ts when timerafter(time+100000) :> time;

		position +=increment;
		c_position_ctrl <: 2;
		c_position_ctrl <: position;
		if(position>300000)
			increment *= -1;
		if(position<0)
			increment *= -1;

	}
}


*/
