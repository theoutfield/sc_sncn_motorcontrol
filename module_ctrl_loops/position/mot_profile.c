
/*
 *
 *
 *  Created on: Oct 2, 2012
 *      Author: Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

unsigned root_function(unsigned uSquareValue)
{
	unsigned uResult;
	float fx;
	fx = uSquareValue;
	fx = sqrt(fx);
	uResult = (unsigned)fx;
	return (uResult);
}

struct pos_params{
float max;
float vi, qi, qf; //user input variables
float dist, d_cruise, t_cruise, tb, tf, t_int, t, ts; //motion profile params
float ai, bi, ci, di, ei, fi, gi; //motion profile constants
float qid, qfd;
float q, qd, q2d;
float samp; float len;
int dirn;
} pos_params_t;

int init(int vel, int pos_i, int pos_f)
{
	int sample;
	pos_params_t.vi =  (float) vel;  //5200*360/60;
	pos_params_t.qi = (float) pos_i; pos_params_t.qf = (float) pos_f;

	pos_params_t.dist= pos_params_t.qf-pos_params_t.qi;
	pos_params_t.dirn = 1;

	if(pos_params_t.dist < 0)
	{
		pos_params_t.dist = - pos_params_t.dist;
		pos_params_t.dirn = -1;
	}

	pos_params_t.d_cruise=.7*pos_params_t.dist;
	pos_params_t.t_cruise = (pos_params_t.d_cruise)/pos_params_t.vi;
	pos_params_t.tb = (pos_params_t.dist - pos_params_t.d_cruise)/pos_params_t.vi;
	pos_params_t.tf = 2*pos_params_t.tb + pos_params_t.t_cruise;

	if(pos_params_t.dirn == -1)
	{
	 pos_params_t.vi = -pos_params_t.vi;
	}

	pos_params_t.ai = pos_params_t.qi;
	pos_params_t.bi = pos_params_t.qid;
	pos_params_t.ci = (pos_params_t.vi - pos_params_t.qid)/(pos_params_t.tb+pos_params_t.tb);
	pos_params_t.di = (pos_params_t.qi+pos_params_t.qf-pos_params_t.vi*pos_params_t.tf)/2;
	pos_params_t.ei = pos_params_t.qf;
	pos_params_t.fi = pos_params_t.qfd;
	pos_params_t.gi = -pos_params_t.ci;
	pos_params_t.samp = pos_params_t.tf/2.0e-3;
	pos_params_t.t_int = pos_params_t.tf/pos_params_t.samp;
	sample = (int)  round(pos_params_t.samp);

	return sample;
}

int mot_q(int i)
{
	int Q;
	pos_params_t.ts =pos_params_t.t_int*i;

	if ( pos_params_t.ts < pos_params_t.tb)
	{
		pos_params_t.q= pos_params_t.ai + pos_params_t.ts*pos_params_t.bi + pos_params_t.ci*pos_params_t.ts*pos_params_t.ts;
	}
	else if (pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts < pos_params_t.tf-pos_params_t.tb)
	{
		pos_params_t.q = pos_params_t.di+pos_params_t.vi*pos_params_t.ts;
	}
	else if ( pos_params_t.tf-pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts <= pos_params_t.tf)
	{
		pos_params_t.q = pos_params_t.ei + (pos_params_t.ts-pos_params_t.tf)*pos_params_t.fi + (pos_params_t.ts-pos_params_t.tf)*(pos_params_t.ts-pos_params_t.tf)*pos_params_t.gi;
	}
	Q = (int) round(pos_params_t.q*1000);
	return Q;
}


int mot_qd(int i)
{
	int Qd;
	pos_params_t.ts =pos_params_t.t_int*i;

	if ( pos_params_t.ts < pos_params_t.tb)
	{
		pos_params_t.qd= pos_params_t.bi + 2*pos_params_t.ts*pos_params_t.ci;
	}
	else if (pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts < pos_params_t.tf-pos_params_t.tb)
	{
		pos_params_t.qd = pos_params_t.vi;
	}
	else if ( pos_params_t.tf-pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts <= pos_params_t.tf)
	{
		pos_params_t.qd = pos_params_t.fi + 2*(pos_params_t.ts-pos_params_t.tf)*pos_params_t.gi;
	}

	Qd = (int) round(pos_params_t.qd);
	return Qd;

}
int mot_q2d(int i)
{
	int Q2d;
	pos_params_t.ts =pos_params_t.t_int*i;

	if ( pos_params_t.ts < pos_params_t.tb)
	{
		pos_params_t.q2d=2*pos_params_t.ci;
	}
	else if (pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts < pos_params_t.tf-pos_params_t.tb)
	{
		pos_params_t.q2d=0;
	}
	else if ( pos_params_t.tf-pos_params_t.tb <= pos_params_t.ts && pos_params_t.ts <= pos_params_t.tf)
	{
		pos_params_t.q2d =   2*pos_params_t.gi;
	}
	Q2d = (int) pos_params_t.q2d;

	return Q2d;

}

