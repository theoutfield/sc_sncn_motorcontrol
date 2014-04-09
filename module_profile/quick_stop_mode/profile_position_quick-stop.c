
/**
 * \file profile_position_quick-stop.c
 * \brief Quick stop Profile Generation for Position
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
	float qi;
	float qf; 								// user input variables

	float ts;
	float ci; 								// motion profile constants

	float tb, t_int; 						// motion profile params

	float q;
	float samp;

	float cur_pos_s;
	float acc;
} pos_param_s;   							// position quick stop parameters



int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_deceleration)  //emergency stop
{
	pos_param_s.qi = 0;
	pos_param_s.qf = (float) actual_velocity;   //always positive -ticks/s

	if(pos_param_s.qf < 0)
		pos_param_s.qf = 0 - pos_param_s.qf;

	pos_param_s.acc = pos_param_s.qf * 10;			//ticks m/s
	if(pos_param_s.acc > max_deceleration)
		pos_param_s.acc = max_deceleration;

	pos_param_s.cur_pos_s = (float) actual_position; //ticks

	pos_param_s.tb = pos_param_s.qf / pos_param_s.acc;

	pos_param_s.ci = - pos_param_s.acc / 2;
	pos_param_s.samp = pos_param_s.tb/1.0e-3;
	pos_param_s.t_int = pos_param_s.tb/pos_param_s.samp;
	if(pos_param_s.samp < 0)
		pos_param_s.samp = 0 - pos_param_s.samp;
	return (int)  round((pos_param_s.samp));
}

int quick_stop_position_profile_generate(int steps, int actual_velocity)
{
	pos_param_s.ts = pos_param_s.t_int * steps;
	pos_param_s.q = pos_param_s.qf * pos_param_s.ts + pos_param_s.ci * pos_param_s.ts * pos_param_s.ts;
    if(actual_velocity >= 0)
    {
    	return (int) round( pos_param_s.cur_pos_s + pos_param_s.q);
    }
    else if(actual_velocity < 0)
    {
	    return (int) round( pos_param_s.cur_pos_s - pos_param_s.q);
    }
    return 0;
}

