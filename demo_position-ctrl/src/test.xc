#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"
#include <profile.h>
#include "position_ctrl_client.h"
#include <torque_ctrl_client.h>
#include <velocity_ctrl_client.h>
#include "comm_loop_client.h"
#include <profile_control.h>
#include <bldc_motor_config.h>
#include <drive_config.h>
#include <xscope.h>

void position_ctrl_unit_test(chanend c_position_ctrl, chanend c_qei, chanend c_hall)
{
	int target_position = 350;			// deg
	int velocity 		= 350;			// rpm
	int acceleration 	= 350;			// rpm/s
	int deceleration 	= 350;     		// rpm/s
	int actual_position;
	ctrl_par position_ctrl_params;
	hall_par hall_params;
	qei_par qei_params;
	in_data d;
	init_qei_param(qei_params);
	init_hall_param(hall_params);


	init_position_profile_limits(MAX_ACCELERATION, MAX_PROFILE_VELOCITY, qei_params, hall_params, SENSOR_USED,\
			MAX_POSITION_LIMIT, MIN_POSITION_LIMIT);

	while(1)
	{
		input_activate(d);
		switch(d.activate)
		{
			case 1:	// Enable Position control
				printstrln("Position control enabled");
				set_position_sensor(SENSOR_USED, c_position_ctrl);
				init_position_control(c_position_ctrl);   // once
				while (1)
				{
					input_pos(d);

					printintln(d.set_position);
					set_profile_position(d.set_position, velocity, acceleration, deceleration, SENSOR_USED, c_position_ctrl);
					if(d.exit_mode == 1)
					{
						printstrln(" set position exit ");
						break;
					}
				}
				break;

			case 0: // Disable Position control
				printstrln("Position control disabled");
				shutdown_position_ctrl(c_position_ctrl);
				break;

			default:
				break;
		}
	}
}

void positioning_accuracy(chanend c_position_ctrl, chanend c_qei, chanend c_hall)
{
	int init_state;
	int actual_position = 0;			// ticks test purpose only
	int target_position = -70997;		// ticks
	int velocity 		= 1000;			// rpm
	int acceleration 	= 1000;			// rpm/s
	int deceleration 	= 1000;     	// rpm/s
	//int turns = 1;
	int tickss= 704512;
	int follow_error;
	timer t; unsigned int time;
	//ctrl_par position_ctrl_params;
	hall_par hall_params;
	qei_par qei_params;
	int position_ramp = 0; int steps; int i;
	int ki = 0;
	init_state = __check_position_init(c_position_ctrl);
	init_qei_param(qei_params);
		init_hall_param(hall_params);
	while(init_state == INIT_BUSY)
	{
		set_position_sensor(SENSOR_USED, c_position_ctrl);
		init_state = init_position_control(c_position_ctrl);
	}

	init_position_profile_limits(MAX_ACCELERATION, MAX_PROFILE_VELOCITY, qei_params, hall_params, \
			SENSOR_USED, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT);

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	//set_profile_position(target_position, velocity, acceleration, deceleration, SENSOR_USED, c_position_ctrl);
	target_position = tickss;
	while(ki <= 10)
	{
		actual_position = get_position(c_position_ctrl);
		steps = init_position_profile(target_position, actual_position, velocity, acceleration, deceleration);

		t :> time;
		for(i = 0; i < steps; i++)
		{
		//	xscope_probe_data(0, position_ramp);
			position_ramp = position_profile_generate(i);
			set_position(position_ramp, c_position_ctrl);
			actual_position = get_position(c_position_ctrl);
			follow_error = position_ramp - actual_position;
			t when timerafter(time + MSEC_STD) :> time;
		//	xscope_int(1, actual_position);
		//	xscope_int(2, follow_error);

		}
		t when timerafter(time + 100*MSEC_STD) :> time;
		//printintln(target_position);
		ki = ki + 1;
		//target_position = target_position - 16000;
		target_position = ki&1;
		if(target_position == 0)
			target_position = tickss;
		else
			target_position = -tickss;

	}
	printintln(target_position);
	actual_position = get_position(c_position_ctrl);
	printintln(actual_position);
//	printstrln("done");
	while(1)
	{
		actual_position = get_position(c_position_ctrl);
		//printintln(actual_position);
		follow_error = target_position - actual_position;
		//xscope_probe_data(0, position_ramp);
		xscope_int(1, actual_position);
		xscope_int(2, follow_error);
		t when timerafter(time + MSEC_STD) :> time;
	}
}

void enable_motor_test(chanend c_commutation)
{
	int i;
	in_data d;
	int ramp = -2800;
	timer t;
	while(1)
	{
		input_shutdown(d);
		if(d.shutdown == 1)
		{
			set_commutation_sinusoidal(c_commutation, 0);
			disable_motor(c_commutation);
			wait_ms(30, 1, t);
		}
		else if(d.shutdown == 0)
		{
			enable_motor(c_commutation);
			wait_ms(30, 1, t);
			i = 0;
			while(1)
			{
				set_commutation_sinusoidal(c_commutation, i);
				i = i-10;
				if(i < ramp)
				{
					i = ramp;
					break;
				}
				wait_ms(10, 1, t);
			}
		}
	}
}

void set_torque_test(chanend c_torque_ctrl, chanend c_velocity_ctrl) {
	int torque;
	in_data d;
	int torque_slope  = 25;
	int acc = 2000;
	cst_par cst_params;
	init_cst_param(cst_params);

	while(1)
	{
		input_mode(d);
		switch(d.mode)
		{
			case 1:
				printstrln("torque mode");
				while (1)
				{
					input_torq(d);

					printintln(d.set_torque);
					set_profile_torque( d.set_torque, torque_slope, cst_params, c_torque_ctrl);
					if(d.exit_mode == 1)
					{
						printstrln(" torque exit ");
						set_profile_torque( 0, torque_slope, cst_params, c_torque_ctrl);
						shutdown_torque_ctrl(c_torque_ctrl);
						break;
					}
				}
				break;

			case 2:
				printstrln("velocity mode");
				while (1)
				{
					input_vel(d);

					printintln(d.set_velocity);
					set_profile_velocity( d.set_velocity, acc, acc, MAX_PROFILE_VELOCITY, c_velocity_ctrl);

					if(d.exit_mode == 1)
					{
						printstrln(" velocity exit ");
						d.set_velocity = 0;
						set_profile_velocity( d.set_velocity , acc, acc, MAX_PROFILE_VELOCITY, c_velocity_ctrl);
						shutdown_velocity_ctrl(c_velocity_ctrl);
						break;
					}
				}
				break;

			default:
				break;
		}
	}

	/*while (1) {
		input_torq(d);
		printintln(d.set_torque);
		//set_profile_torque( d.set_torque, torque_slope, cst_params, c_torque_ctrl);

	}*/
}
