#include "position_ctrl.h"
#include "refclk.h"
#include "comm_loop.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"
#include <profile.h>
#include <profile_control.h>

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration, chanend c_position_ctrl)
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
		/*if(init_state == INIT)
			printstrln("position control intialized");
		else
			printstrln("intialize position control failed");*/
	}

	if(init_state == INIT)
	{
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
