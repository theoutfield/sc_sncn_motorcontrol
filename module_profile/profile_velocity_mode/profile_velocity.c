
/**
 *
 * \file profile_velocity.c
 *
 * \brief Profile Generation for Velocity
 * 	Implements velocity profile based on linear functions.
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
	profile_vel_params.acc = (float) acceleration;
	profile_vel_params.dec = (float) deceleration;

	if(target_velocity >= max_velocity)
		target_velocity = max_velocity;
	else if(target_velocity <= -max_velocity)
		target_velocity = 0-max_velocity;
	profile_vel_params.v_d = (float) target_velocity;
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


	profile_vel_params.s_time = profile_vel_params.t/profile_vel_params.T;

	return (int) round (profile_vel_params.T);
}

int velocity_profile_generate(int step)
{
   return (int) round( profile_vel_params.u + profile_vel_params.a_d * profile_vel_params.s_time * step);
}

