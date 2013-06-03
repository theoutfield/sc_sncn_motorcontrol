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

void position_control(chanend c_torque, chanend c_hall_p4, chanend c_position_ctrl)
{
	int actual_position = 0;
	timer ts;
	unsigned int time;
	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;
	int Kp = 6, Kd = 0, Ki = 0;
	int max_integral = (13739)/1;
	int target_position = 15000;
	int in_cmd;
	//get_hall_absolute_pos(chanend c_hall)

	ts:> time;
	ts when timerafter(time+3*SEC_FAST) :> time;
	//set_commutation(c_torque, 1000);
	while(1)
	{
		select{
			case c_position_ctrl :> in_cmd:
				c_position_ctrl :> target_position;
				break;
			default:
				break;
		}

		ts when timerafter(time+100000) :> time; //1khz

		actual_position = get_hall_absolute_pos(c_hall_p4);

		//xscope_probe_data(0, actual_position);

		error_position = (target_position - actual_position)*1000;
		error_position_I = error_position_I + error_position;
		error_position_D = error_position - previous_error;

		if(error_position_I > max_integral*1000)
			error_position_I = max_integral*1000;
		else if(error_position_I < -max_integral*1000)
			error_position_I = 0 - max_integral*1000;

		position_control_out = (Kp*error_position)/10000 + (Ki*error_position_I) + (Kd*error_position_D);

		if(position_control_out > 13739)
			position_control_out = 13739;
		else if(position_control_out < -13739)
			position_control_out = 0-13739;

		//set_torque(c_torque, speed_control_out);
		set_commutation(c_torque, position_control_out);

		#ifdef ENABLE_xscope_main
		xscope_probe_data(0, actual_position);
		xscope_probe_data(1, target_position);
		#endif

		previous_error = error_position;

	}
}
*/
