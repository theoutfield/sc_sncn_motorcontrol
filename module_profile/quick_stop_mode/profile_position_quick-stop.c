/*
 * profile_position_quick-stop.c
 *
 *  Created on: Sep 3, 2013
 *      Author: pkanajar
 */
#include <profile.h>

struct pos_params
{
	float max;												// shaft output speed in deg/s

	float vi;
	float qi;
	float qf; 												// user input variables

	float  t;
	float ts;

	int dirn;

	float ai, bi, ci, di, ei, fi, gi; 						// motion profile constants

	float qid, qfd;

	float dist, d_cruise;

	float t_cruise, tb, tf, t_int; 							// motion profile params


	float q, qd, q2d;
	float samp; float len;

	float cur_pos_s;
	float acc;
	int negative_s;
} pos_param_s;   											// position quick stop parameters



int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration)  //emergency stop
{
	pos_param_s.qi = 0;
	pos_param_s.qf = (float) actual_velocity;   //always positive

	 if(pos_param_s.qf < 0)
		 pos_param_s.qf = 0 - pos_param_s.qf;

	 pos_param_s.acc = pos_param_s.qf * 8;  // 8 times deceleration
	 if(pos_param_s.acc > max_acceleration)
		 pos_param_s.acc = max_acceleration;

	 pos_param_s.cur_pos_s = (float) actual_position;

	 pos_param_s.tb = pos_param_s.qf / pos_param_s.acc;

	 pos_param_s.ci = - pos_param_s.acc / 2;
	 pos_param_s.samp = pos_param_s.tb/1.0e-3;
	 pos_param_s.t_int = pos_param_s.tb/pos_param_s.samp;
	 if(pos_param_s.samp<0)
		 pos_param_s.samp= 0 - pos_param_s.samp;
	 return (int)  round((pos_param_s.samp));
}

int quick_stop_position_profile_generate(int steps, int actual_velocity)
{
	pos_param_s.ts = pos_param_s.t_int*steps;
	pos_param_s.q = pos_param_s.qf * pos_param_s.ts + pos_param_s.ci*pos_param_s.ts*pos_param_s.ts;
    if(actual_velocity >= 0)
    {
    	return (int) round( pos_param_s.cur_pos_s + pos_param_s.q*10000.0f);
    }
    else if(actual_velocity < 0)
    {
	    return (int) round( pos_param_s.cur_pos_s - pos_param_s.q*10000.0f);
    }
    return 0;
}

