/*Profile Velocity Quick Stop*/

#include <stdio.h>

int i;
float samp, T;
float t, t_stamp;
float v_d = 0, a_d, u;
int v_ramp[2000];
int length;


struct QUICK_STOP_VELOCITY_PARAM
{
	float max_deceleration;									// max allowed deceleration
	float u, a_d, t; 										// user input variables
	float T;
	float samp;
	int length;
} qstop_vel_params;

void init(int actual_velocity, int quick_stop_deceleration)
{
	qstop_vel_params.u = (float) actual_velocity;

	if(quick_stop_deceleration < 0)
		quick_stop_deceleration = 0 - quick_stop_deceleration;

	qstop_vel_params.a_d = (float) quick_stop_deceleration;

	qstop_vel_params.t = 0 - qstop_vel_params.u/qstop_vel_params.a_d;		//default reduce velocity to zero  (v_d - u)/a_d;

	qstop_vel_params.samp = 0.001; 											//
	qstop_vel_params.T = qstop_vel_params.t/qstop_vel_params.samp;

    if(qstop_vel_params.T<0)
    	qstop_vel_params.T = 0 - qstop_vel_params.T;

    qstop_vel_params.length = (int) round (qstop_vel_params.T);
    qstop_vel_params.samp = qstop_vel_params.t/qstop_vel_params.T;

	return;
}
//t_stamp = 0;
 //  for(i = 1; i < length; i++)

int quick_stop_velocity_profile_generate(int step)
{
   return (int) round( qstop_vel_params.u + qstop_vel_params.a_d * qstop_vel_params.samp * step);
}
