/*Profile Velocity Mode*/

#include <profile.h>
#include <stdio.h>


struct PROFILE_VELOCITY_PARAM
{
	float max_acceleration, max_deceleration;	// max allowed acceleration & deceleration
	float acc, dec;								// acceleration & deceleration input
	float u;									// initial velocity
	float v_d;									// desired velocity
	float a_d;									// desired acceleration
	float t; 									// time
	float T;									// total no. of Samples
	float s_time;								// sampling time
	int oldst;									// old state of acc/dec
} profile_vel_params;

int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration, int max_velocity)
{
	profile_vel_params.u = (float) actual_velocity;
//	profile_vel_params.v_d = (float) target_velocity;
	profile_vel_params.acc = (float) acceleration;
	profile_vel_params.dec = (float) deceleration;

	if(target_velocity >= max_velocity)
		target_velocity = max_velocity;
	else if(target_velocity <= -max_velocity)
		target_velocity = 0-max_velocity;
	profile_vel_params.v_d = (float) target_velocity;
	//printf("srta");
//printf("\n%d\n",target_velocity);
	/*both initial and desired velocity - positive case*/
    if(profile_vel_params.u>=0 && profile_vel_params.v_d >=0)
    {
    	if(profile_vel_params.v_d >= profile_vel_params.u)
    	{
    		profile_vel_params.a_d = profile_vel_params.acc;
    		profile_vel_params.oldst= 1;
    	}
    	else if(profile_vel_params.v_d < profile_vel_params.u)
    	{
    		profile_vel_params.a_d = profile_vel_params.dec;
    		profile_vel_params.oldst= 2;
    	}
    }
    /*both initial and desired velocity - negative case*/
    else if(profile_vel_params.u<=0 && profile_vel_params.v_d <=0)
    {
    	if(profile_vel_params.u==0)
    	{
    		profile_vel_params.a_d = profile_vel_params.acc;
    		profile_vel_params.oldst= -1;
    	}
    	else if(profile_vel_params.v_d==0)
    	{
    		profile_vel_params.a_d = -profile_vel_params.dec;
    		profile_vel_params.oldst= -2;
    	}
    	else
    	{
    		if(profile_vel_params.v_d < profile_vel_params.u)
    		{
    			profile_vel_params.a_d = -profile_vel_params.acc;
    			profile_vel_params.oldst= -1;
    		}
    		else if(profile_vel_params.v_d > profile_vel_params.u)
    		{
    			profile_vel_params.a_d = -profile_vel_params.dec;
    			profile_vel_params.oldst= -2;
    		}
    	}
    }
    /*initial and desired velocity - transition from +ve to -ve case*/
    else if(profile_vel_params.u>0 && profile_vel_params.v_d<0)
    {
    	if(profile_vel_params.oldst==2 || profile_vel_params.oldst==-2)
    	{
    		profile_vel_params.a_d = profile_vel_params.acc;
    		profile_vel_params.oldst= 1;
    	}
    	else
    	{
    		profile_vel_params.a_d = profile_vel_params.dec;
    		profile_vel_params.oldst= 2;
    	}
    }
    /*initial and desired velocity - transition from -ve to +ve case*/
    else if(profile_vel_params.u<0 && profile_vel_params.v_d>0)
    {
    	if(profile_vel_params.oldst==-1)
    	{
    		profile_vel_params.a_d = -profile_vel_params.dec;
    		profile_vel_params.oldst=-2;
    	}
    	else
    	{
    		profile_vel_params.a_d = -profile_vel_params.acc;
    		profile_vel_params.oldst= -1;
    	}
    }

    profile_vel_params.s_time = .001f;

    // compute time needed
    profile_vel_params.t = (profile_vel_params.v_d - profile_vel_params.u)/profile_vel_params.a_d;

    profile_vel_params.T = profile_vel_params.t/profile_vel_params.s_time;

	if(profile_vel_params.T<0)
		profile_vel_params.T = -profile_vel_params.T;

	//length = (int) round (T);
	profile_vel_params.s_time = profile_vel_params.t/profile_vel_params.T;

	return (int) round (profile_vel_params.T);
}

int velocity_profile_generate(int step)
{
   return (int) round( profile_vel_params.u + profile_vel_params.a_d * profile_vel_params.s_time * step);
}

