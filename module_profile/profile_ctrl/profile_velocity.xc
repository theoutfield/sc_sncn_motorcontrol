#include "velocity_ctrl.h"
#include "refclk.h"
#include "comm_loop.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"
#include <profile.h>
#include <profile_control.h>
#define debug_print


void set_profile_velocity(int target_velocity, int acceleration, int deceleration, int max_profile_velocity, chanend c_velocity_ctrl)
{
	int actual_velocity;
	timer t;
	unsigned int time;
	int steps = 0;
	int velocity_ramp;
	int i;
	int init_state = __check_velocity_init(c_velocity_ctrl);
	if(init_state == INIT_BUSY)
	{
		init_state = init_velocity_control(c_velocity_ctrl);
		/*if(init_state == INIT)
			printstrln("velocity control intialized");
		else
			printstrln("intialize velocity control failed");*/
	}


	if(init_state == INIT)
	{
		actual_velocity = get_velocity(c_velocity_ctrl);
		steps = init_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration, max_profile_velocity);
		t :> time;
		for(i = 1; i < steps; i++)
		{
			velocity_ramp = velocity_profile_generate(i);
			set_velocity(velocity_ramp, c_velocity_ctrl);
			actual_velocity = get_velocity(c_velocity_ctrl);//

			t when timerafter(time + MSEC_STD) :> time;

			/*xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1, velocity_ramp);*/
		}
		t when timerafter(time + 30 * MSEC_STD) :> time;
	}
}
