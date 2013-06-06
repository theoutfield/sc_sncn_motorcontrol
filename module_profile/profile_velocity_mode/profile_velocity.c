/*Profile Velocity Mode*/

#include <stdio.h>
#include <math.h>

int i;
float samp, T;
float t, t_stamp;
float vel, acc, u, dec;
float v_d, a_d;
int v_ramp[20000];
int length;
int oldst =  -2;

int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration)
{
	u = (float) actual_velocity;
	vel = (float) target_velocity;
	acc = (float) acceleration;
	dec = (float) deceleration;

	v_d = vel;

    if(u>=0 && v_d >=0)
    {
    	if(v_d >= u)
    	{
    		a_d = acc;
    		oldst= 1;
    	}
    	else if(v_d < u)
    	{
    		a_d = dec;
    		oldst= 2;
    	}
    }
    else if(u<=0 && v_d <=0)
    {
    	if(u==0)
    	{
    		a_d = acc;
    		oldst= -1;
    	}
    	else if(v_d==0)
    	{
    		a_d = -dec;
    		oldst= -2;
    	}
    	else
    	{
    		if(v_d < u)
    		{
    			a_d = -acc;
    			oldst= -1;
    		}
    		else if(v_d > u)
    		{
    			a_d = -dec;
    			oldst= -2;
    		}
    	}
    }
    else if(u>0 && v_d<0)
    {
    	if(oldst==2 || oldst==-2)
    	{
    		a_d = acc;
    		oldst= 1;
    	}
    	else
    	{
    		a_d = dec;
    		oldst= 2;
    	}
    }
    else if(u<0 && v_d>0)
    {
    	if(oldst==-1)
    	{
    		a_d = -dec;
    		oldst=-2;
    	}
    	else
    	{
    		a_d = -acc;
    		oldst= -1;
    	}
    }

    // compute time needed
    t = (v_d - u)/a_d;

    T = t/samp;

	if(T<0)
	   T = -T;

	//length = (int) round (T);
	samp = t/T;

	return (int) round (T);
}

int velocity_profile_generate(int step)
{
   return (int) round( u + a_d * samp * step);
}

