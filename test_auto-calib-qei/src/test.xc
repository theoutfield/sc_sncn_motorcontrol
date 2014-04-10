#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"
#include <torque_ctrl_client.h>
#include <velocity_ctrl_client.h>
#include "comm_loop_client.h"
#include <profile_control.h>
#include <bldc_motor_config.h>

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
