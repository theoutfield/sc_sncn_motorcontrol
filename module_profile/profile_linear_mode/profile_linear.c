
/**
 *
 * \file profile_linear.c
 *
 * \brief Profile Generation
 * 	Implements Torque profile based on linear functions.
 *
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */



#include <profile.h>
#include <stdio.h>


struct PROFILE_LINEAR_PARAM
{
	float max_acceleration, max_deceleration;	// max allowed acceleration & deceleration
	float acc, dec;								// acceleration & deceleration input
	float u;									// initial value
	float v_d;									// desired value
	float a_d;									// desired acceleration
	float t; 									// time
	float T;									// total no. of Samples
	float s_time;								// sampling time
	int oldst;									// old state of acc/dec
} profile_linear_params;

int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration, int max_value)
{
	profile_linear_params.u = (float) actual_value;

	profile_linear_params.acc = (float) acceleration;
	profile_linear_params.dec = (float) deceleration;

	if(target_value >= max_value)
		target_value = max_value;
	else if(target_value <= -max_value)
		target_value = 0-max_value;
	profile_linear_params.v_d = (float) target_value;

	/*both initial and desired velocity - positive case*/
    if(profile_linear_params.u>=0 && profile_linear_params.v_d >=0)
    {
    	if(profile_linear_params.v_d >= profile_linear_params.u)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= 1;
    	}
    	else if(profile_linear_params.v_d < profile_linear_params.u)
    	{
    		profile_linear_params.a_d = profile_linear_params.dec;
    		profile_linear_params.oldst= 2;
    	}
    }
    /*both initial and desired velocity - negative case*/
    else if(profile_linear_params.u<=0 && profile_linear_params.v_d <=0)
    {
    	if(profile_linear_params.u==0)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= -1;
    	}
    	else if(profile_linear_params.v_d==0)
    	{
    		profile_linear_params.a_d = -profile_linear_params.dec;
    		profile_linear_params.oldst= -2;
    	}
    	else
    	{
    		if(profile_linear_params.v_d < profile_linear_params.u)
    		{
    			profile_linear_params.a_d = -profile_linear_params.acc;
    			profile_linear_params.oldst= -1;
    		}
    		else if(profile_linear_params.v_d > profile_linear_params.u)
    		{
    			profile_linear_params.a_d = -profile_linear_params.dec;
    			profile_linear_params.oldst= -2;
    		}
    	}
    }
    /*initial and desired velocity - transition from +ve to -ve case*/
    else if(profile_linear_params.u>0 && profile_linear_params.v_d<0)
    {
    	if(profile_linear_params.oldst==2 || profile_linear_params.oldst==-2)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= 1;
    	}
    	else
    	{
    		profile_linear_params.a_d = profile_linear_params.dec;
    		profile_linear_params.oldst= 2;
    	}
    }
    /*initial and desired velocity - transition from -ve to +ve case*/
    else if(profile_linear_params.u<0 && profile_linear_params.v_d>0)
    {
    	if(profile_linear_params.oldst==-1)
    	{
    		profile_linear_params.a_d = -profile_linear_params.dec;
    		profile_linear_params.oldst=-2;
    	}
    	else
    	{
    		profile_linear_params.a_d = -profile_linear_params.acc;
    		profile_linear_params.oldst= -1;
    	}
    }

    profile_linear_params.s_time = .001f;

    // compute time needed
    profile_linear_params.t = (profile_linear_params.v_d - profile_linear_params.u)/profile_linear_params.a_d;

    profile_linear_params.T = profile_linear_params.t/profile_linear_params.s_time;

	if(profile_linear_params.T<0)
		profile_linear_params.T = -profile_linear_params.T;


	profile_linear_params.s_time = profile_linear_params.t/profile_linear_params.T;

	return (int) round (profile_linear_params.T);
}

int  linear_profile_generate(int step)
{
   return (int) round( profile_linear_params.u + profile_linear_params.a_d * profile_linear_params.s_time * step);
}

int init_linear_profile_float(float target_value, float actual_value, float acceleration, float deceleration, float max_value)
{
	profile_linear_params.u =  actual_value;

	profile_linear_params.acc = acceleration;
	profile_linear_params.dec = deceleration;

	if(target_value >= max_value)
		target_value = max_value;
	else if(target_value <= -max_value)
		target_value = 0-max_value;
	profile_linear_params.v_d = target_value;

	/*both initial and desired velocity - positive case*/
    if(profile_linear_params.u>=0 && profile_linear_params.v_d >=0)
    {
    	if(profile_linear_params.v_d >= profile_linear_params.u)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= 1;
    	}
    	else if(profile_linear_params.v_d < profile_linear_params.u)
    	{
    		profile_linear_params.a_d = profile_linear_params.dec;
    		profile_linear_params.oldst= 2;
    	}
    }
    /*both initial and desired velocity - negative case*/
    else if(profile_linear_params.u<=0 && profile_linear_params.v_d <=0)
    {
    	if(profile_linear_params.u==0)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= -1;
    	}
    	else if(profile_linear_params.v_d==0)
    	{
    		profile_linear_params.a_d = -profile_linear_params.dec;
    		profile_linear_params.oldst= -2;
    	}
    	else
    	{
    		if(profile_linear_params.v_d < profile_linear_params.u)
    		{
    			profile_linear_params.a_d = -profile_linear_params.acc;
    			profile_linear_params.oldst= -1;
    		}
    		else if(profile_linear_params.v_d > profile_linear_params.u)
    		{
    			profile_linear_params.a_d = -profile_linear_params.dec;
    			profile_linear_params.oldst= -2;
    		}
    	}
    }
    /*initial and desired velocity - transition from +ve to -ve case*/
    else if(profile_linear_params.u>0 && profile_linear_params.v_d<0)
    {
    	if(profile_linear_params.oldst==2 || profile_linear_params.oldst==-2)
    	{
    		profile_linear_params.a_d = profile_linear_params.acc;
    		profile_linear_params.oldst= 1;
    	}
    	else
    	{
    		profile_linear_params.a_d = profile_linear_params.dec;
    		profile_linear_params.oldst= 2;
    	}
    }
    /*initial and desired velocity - transition from -ve to +ve case*/
    else if(profile_linear_params.u<0 && profile_linear_params.v_d>0)
    {
    	if(profile_linear_params.oldst==-1)
    	{
    		profile_linear_params.a_d = -profile_linear_params.dec;
    		profile_linear_params.oldst=-2;
    	}
    	else
    	{
    		profile_linear_params.a_d = -profile_linear_params.acc;
    		profile_linear_params.oldst= -1;
    	}
    }

    profile_linear_params.s_time = .001f;

    // compute time needed
    profile_linear_params.t = (profile_linear_params.v_d - profile_linear_params.u)/profile_linear_params.a_d;

    profile_linear_params.T = profile_linear_params.t/profile_linear_params.s_time;

	if(profile_linear_params.T<0)
		profile_linear_params.T = -profile_linear_params.T;


	profile_linear_params.s_time = profile_linear_params.t/profile_linear_params.T;

	return (int) round (profile_linear_params.T);
}

float linear_profile_generate_float(int step)
{
   return  profile_linear_params.u + profile_linear_params.a_d * profile_linear_params.s_time * (float)step;
}
