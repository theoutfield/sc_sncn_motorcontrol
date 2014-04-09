
/**
 * \file profile_velocity_quick-stop.c
 * \brief Quick stop Profile Generation for Velocity
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
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

struct
{
	float max_deceleration;		// max allowed deceleration
	float u;					// initial velocity
	float a_d;					// desired acceleration
	float t; 					// time
	float T;					// total no. of Samples
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


int quick_stop_velocity_profile_generate(int step)
{
   return (int) round( qstop_vel_params.u + qstop_vel_params.a_d * qstop_vel_params.s_time * step);
}
