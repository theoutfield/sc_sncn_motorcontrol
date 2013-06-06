/*Profile Velocity Quick Stop*/

#include <stdio.h>
#include <math.h>



struct QUICK_STOP_VELOCITY_PARAM
{
	float max_deceleration;		// max allowed deceleration
	float u;					// initial velocity
	float a_d;					// desired acceleration
	float t; 					// time
	float T;					// total of Samples
	float s_time;				// sampling time
} qstop_vel_params;

int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration)
{

	qstop_vel_params.u = (float) actual_velocity;

	if(quick_stop_deceleration < 0)
		quick_stop_deceleration = 0 - quick_stop_deceleration;

	qstop_vel_params.a_d = (float) quick_stop_deceleration;

	qstop_vel_params.t = 0 - qstop_vel_params.u/qstop_vel_params.a_d;		//default reduce velocity to zero  (v_d - u)/a_d;

	qstop_vel_params.s_time = 0.001; 											//
	qstop_vel_params.T = qstop_vel_params.t/qstop_vel_params.s_time;

    if(qstop_vel_params.T<0)
    	qstop_vel_params.T = 0 - qstop_vel_params.T;

    qstop_vel_params.s_time = qstop_vel_params.t/qstop_vel_params.T;

	return (int) round (qstop_vel_params.T);
}
//t_stamp = 0;
 //  for(i = 1; i < length; i++)

int quick_stop_velocity_profile_generate(int step)
{
   return (int) round( qstop_vel_params.u + qstop_vel_params.a_d * qstop_vel_params.s_time * step);
}
