#include "torque_ctrl.h"
#include "refclk.h"
#include "comm_loop.h"
#include <xscope.h>
#include <internal_config.h>
#include <drive_config.h>
#include "print.h"
#include <profile.h>
#include <profile_control.h>


void set_profile_torque(int target_torque, int torque_slope, cst_par &cst_params, chanend c_torque_ctrl)
{
	int i;
	int steps;
	int torque_ramp;
	int actual_torque;
	timer t;
	unsigned int time;
	int init = INIT_BUSY;

	int init_state = __check_torque_init(c_torque_ctrl);
	if(init_state == INIT_BUSY)
	{
		init = init_torque_control(c_torque_ctrl);
		/*if(init == INIT)
		{
			printstrln("torque control intialized");
		}*/
	}

	actual_torque = get_torque(cst_params , c_torque_ctrl)*cst_params.polarity;
	steps = init_linear_profile(target_torque, actual_torque, torque_slope, torque_slope, cst_params.max_torque);
	t:>time;
	for(i = 1; i<steps; i++)
	{
		torque_ramp =  linear_profile_generate(i);
		set_torque( torque_ramp, cst_params , c_torque_ctrl);
		actual_torque = get_torque(cst_params , c_torque_ctrl)*cst_params.polarity;
		t when timerafter(time + MSEC_STD) :> time;
		/*xscope_probe_data(0, torque_ramp);
		xscope_probe_data(1, actual_torque);*/
	}
	t when timerafter(time + 30 * MSEC_STD) :> time;
}
