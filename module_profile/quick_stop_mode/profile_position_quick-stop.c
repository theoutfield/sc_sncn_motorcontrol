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
	int negative_s;
} pos_param_s;   											// position quick stop parameters




int init_stop(int c_vel, int c_pos)  //emergency stop
{
	pos_param_s.qi = 0;
	pos_param_s.qf = (float) c_vel;   //always positive

	 if(pos_param_s.qf<0)
		 pos_param_s.qf = 0-pos_param_s.qf;


	 pos_param_s.cur_pos_s = (float) c_pos;

	 pos_param_s.vi =   pos_param_s.qf;
	 pos_param_s.qid = pos_param_s.qi; pos_param_s.qfd = pos_param_s.qf;
	 pos_param_s.dist= pos_param_s.qf-pos_param_s.qi;

	 pos_param_s.dirn = 1;
	 if(pos_param_s.dist < 0)
	 {
		 pos_param_s.dist = - pos_param_s.dist;
		 pos_param_s.dirn = -1;
	 }

	 pos_param_s.tb = (pos_param_s.dist*0.3)/pos_param_s.vi/2;

	 if(pos_param_s.dirn == -1)
	 {
	    pos_param_s.vi = -pos_param_s.vi;
	 }



	 pos_param_s.ai = pos_param_s.qi;
	 pos_param_s.bi = pos_param_s.qid;
	 pos_param_s.ci = (pos_param_s.vi - pos_param_s.qid)/(pos_param_s.tb+pos_param_s.tb);


	 pos_param_s.samp = pos_param_s.tb/1.0e-3;
	 pos_param_s.t_int = pos_param_s.tb/pos_param_s.samp;
	 if(pos_param_s.samp<0)
		 pos_param_s.samp= 0 - pos_param_s.samp;
	 return (int)  round((pos_param_s.samp));
}

int mot_q_stop(int i, int c_vel)
{
	pos_param_s.ts = pos_param_s.t_int*i;
	pos_param_s.q = pos_param_s.ai + pos_param_s.ts*pos_param_s.bi + pos_param_s.ci*pos_param_s.ts*pos_param_s.ts;
    if(c_vel >= 0)
    {
    	return (int) round( pos_param_s.cur_pos_s + pos_param_s.q*1000);
    }
    else if(c_vel < 0)
    {
	    return (int) round( pos_param_s.cur_pos_s - pos_param_s.q*1000);
    }
    return 0;
}

