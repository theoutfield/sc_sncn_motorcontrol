#include "velocity_ctrl.h"
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop.h"
#include "filter_blocks.h"
#include <xscope.h>
#include "print.h"

#define Debug_velocity_ctrl
//default runs on CORE 2/CORE 1/CORE 0
#define HALL 1
#define QEI 2

#define VELOCITY_Kp_NUMERATOR 	 5
#define VELOCITY_Kp_DENOMINATOR  10
#define VELOCITY_Ki_NUMERATOR    5
#define VELOCITY_Ki_DENOMINATOR  100
#define VELOCITY_Kd_NUMERATOR   0
#define VELOCITY_Kd_DENOMINATOR 1
#define VELOCITY_CONTROL_LOOP_TIME 1			//in ms

#define FILTER_SIZE 8                           //default


#define FILTER_SIZE_MAX 16
#define SET_VELOCITY_TOKEN 50
#define GET_VELOCITY_TOKEN 60

void init_velocity_control(ctrl_par &velocity_ctrl_params)
{
	velocity_ctrl_params.Kp_n = VELOCITY_Kp_NUMERATOR;
	velocity_ctrl_params.Kp_d = VELOCITY_Kp_DENOMINATOR;
	velocity_ctrl_params.Ki_n = VELOCITY_Ki_NUMERATOR;
	velocity_ctrl_params.Ki_d = VELOCITY_Ki_DENOMINATOR;
	velocity_ctrl_params.Kd_n = VELOCITY_Kd_NUMERATOR;
	velocity_ctrl_params.Kd_d = VELOCITY_Kd_DENOMINATOR;
	velocity_ctrl_params.Loop_time = VELOCITY_CONTROL_LOOP_TIME * MSEC_STD;  //units - core timer value //CORE 2/1/0

	velocity_ctrl_params.Control_limit = 13739; //default

	if(velocity_ctrl_params.Ki_n != 0)    							//auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = (velocity_ctrl_params.Control_limit * velocity_ctrl_params.Ki_d)/velocity_ctrl_params.Ki_n ;
	else
		velocity_ctrl_params.Integral_limit = 0;

	return;
}

void init_sensor_filter(filt_par &sensor_filter_par) //optional for user to change
{
	sensor_filter_par.filter_length = FILTER_SIZE;
	return;
}

void velocity_control(ctrl_par &velocity_ctrl_params, filt_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params, \
		 	 	 	 	 int sensor_used, chanend c_hall, chanend c_qei, chanend c_velocity_ctrl, chanend c_commutation)
{
	/* Controller declarations */
	int actual_velocity = 0;
	int target_velocity = 0;
	int error_velocity = 0;
	int error_velocity_D = 0;
	int error_velocity_I = 0;
	int previous_error = 0;
	int velocity_control_out = 0;

	timer ts;
	unsigned int time;

	/* Sensor filter declarations */
	int filter_length = sensor_filter_params.filter_length;
	int filter_buffer[FILTER_SIZE_MAX];						//default size used at compile time (cant be changed further)
	int index = 0;
	int filter_output;
	int old_filter_output = 0;

	/* speed calc declarations */
	int pos;
	int init = 0;
	int prev = 0;
	int cal_speed = 0;			// rpm
	int diff;
	int dirn = 0;
	int old;
	int cal_speed_n = 1000*60; // constant
	int cal_speed_d_hall = hall_params.pole_pairs*4095*(velocity_ctrl_params.Loop_time/MSEC_STD); // variable pole_pairs    core 2/1/0 only
	int cal_speed_d_qei = qei_params.real_counts*(velocity_ctrl_params.Loop_time/MSEC_STD);		  // variable qei_real_max  core 2/1/0 only

	int cmd;

	init_filter(filter_buffer, index, filter_length);


	while(1)
	{
	  unsigned cmd, found =0;
	  select
	  {
		case c_commutation :> cmd:
			found = 1;
			break;
		default:
			break;
	  }
	  if(found == 1)

		  break;
	}

	printstrln("start vel");
	ts :> time;
	ts when timerafter(time+1*SEC_FAST) :> time;


	while(1)
	{
		#pragma ordered
		select
		{
			case ts when timerafter(time + velocity_ctrl_params.Loop_time) :> time:


				/* acq actual velocity hall/qei with filter*/

				if(init == 0)
				{
					//set_commutation_sinusoidal(c_commutation, 400);
					pos = get_hall_absolute_pos(c_hall);
					if(pos > 2049)
					{
						init = 1;
						prev = 2049;
					}
					else if(pos < -2049)
					{
						init = 1;
						prev = -2049;
					}
					cal_speed = 0;

					//target_velocity = 0;
				}
				if(sensor_used == HALL)
				{
					pos = get_hall_absolute_pos(c_hall);

					if(init == 1)
					{
						diff = pos - prev;
						if(diff > 50000) diff = old;
						else if(diff < -50000) diff = old;
						cal_speed = (diff*cal_speed_n)/cal_speed_d_hall;
		#ifdef Debug_velocity_ctrl
						xscope_probe_data(0, cal_speed);
		#endif
						prev = pos;
						old = diff;
					}
				}
				else if(sensor_used == QEI)
				{
					{pos, dirn} = get_qei_position_count(c_qei);
					diff = pos - prev;
					if(diff > 3080) diff = old;
					if(diff < -3080) diff = old;
					cal_speed = (diff*cal_speed_n)/cal_speed_d_qei;

		#ifdef Debug_velocity_ctrl
					xscope_probe_data(0, cal_speed);
		#endif

					prev = pos;
					old = diff;
				}



				actual_velocity = filter(filter_buffer, index, filter_length, cal_speed);


		#ifdef Debug_velocity_ctrl
				xscope_probe_data(1, actual_velocity);
		#endif

				/* Controller */
				error_velocity   = (target_velocity - actual_velocity);
				error_velocity_I = error_velocity_I + error_velocity;
				error_velocity_D = error_velocity - previous_error;

				if(error_velocity_I > (velocity_ctrl_params.Integral_limit))
					error_velocity_I = (velocity_ctrl_params.Integral_limit);
				else if(error_velocity_I < -(velocity_ctrl_params.Integral_limit))
					error_velocity_I = 0 -(velocity_ctrl_params.Integral_limit);

velocity_control_out = (velocity_ctrl_params.Kp_n*error_velocity)/(velocity_ctrl_params.Kp_d) + (velocity_ctrl_params.Ki_n*error_velocity_I)/(velocity_ctrl_params.Ki_d) \
										+ (velocity_ctrl_params.Kd_n*error_velocity_D)/(velocity_ctrl_params.Kd_d);

				if(velocity_control_out > velocity_ctrl_params.Control_limit)
					velocity_control_out = velocity_ctrl_params.Control_limit;
				else if(velocity_control_out < -velocity_ctrl_params.Control_limit)
					velocity_control_out = 0 - velocity_ctrl_params.Control_limit;



				set_commutation_sinusoidal(c_commutation, velocity_control_out);

if(actual_velocity == 0)
{
//	set_commutation_sinusoidal(c_commutation, velocity_control_out)
	c_commutation<:3;
	c_commutation <: 30;
	ts when timerafter(time + velocity_ctrl_params.Loop_time/2) :> time;
}

				previous_error = error_velocity;




				break;

				/* acq target velocity etherCAT */
			case c_velocity_ctrl :> cmd:
				if(cmd == SET_VELOCITY_TOKEN)
					c_velocity_ctrl :> target_velocity;

				else if(cmd == GET_VELOCITY_TOKEN)
					c_velocity_ctrl <: actual_velocity;
				break;

		}



	}


}
