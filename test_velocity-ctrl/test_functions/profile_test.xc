#include "profile_test.h"
void test_quick_stop_velocity()
{
	int i;
	int u = -240, a_d =400, steps = 0, v_ramp;
	timer t; int time;
	steps = init_quick_stop_velocity_profile(u, a_d);

	printintln(steps);

	t:>time;					//t_stamp = 0;
	t when timerafter(time + 2*SEC_FAST) :> time;
	printstrln("start");
	for(i = 1; i < steps; i++)
	{

		v_ramp = quick_stop_velocity_profile_generate(i);
		xscope_probe_data(0, v_ramp);
	}
	printstrln("end");
	while(1);
}
void test_profile_velocity()
{
	int i;
	int u = 0, v_d = -200, acc = 80, dec =50;
	int steps = 0, v_ramp;
	timer t; int time;
	steps = init_velocity_profile(v_d, u, acc, dec);

	printintln(steps);

	t:>time;					//t_stamp = 0;
	t when timerafter(time + 2*SEC_FAST) :> time;
	printstrln("start");
	for(i = 1; i < steps; i++)
	{

		v_ramp = velocity_profile_generate(i);
		xscope_probe_data(0, v_ramp);
	}

	u = -200; v_d = 200;
	steps = init_velocity_profile(v_d, u, acc, dec);

	for(i = 1; i < steps; i++)
	{

		v_ramp = velocity_profile_generate(i);
		xscope_probe_data(0, v_ramp);
	}


	u = 200; v_d = -200;
	steps = init_velocity_profile(v_d, u, acc, dec);

	for(i = 1; i < steps; i++)
	{

		v_ramp = velocity_profile_generate(i);
		xscope_probe_data(0, v_ramp);
	}


	printstrln("end");
	while(1);
}
