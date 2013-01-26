
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

float max = 1000;
float vi, qi, qf; //user input variables
float dist, d_cruise, t_cruise, tb, tf, t_int, t, ts=0; //motion profile params
float ai, bi, ci, di, ei, fi, gi; //motion profile constants
float qid=0.0, qfd=0.0;
float q, qd, q2d;
float samp; float len;
int dirn = 1;

int init(int vel, int pos_i, int pos_f)
{
	 vi =  (float) vel;  //5200*360/60;
	 qi = (float) pos_i; qf = (float) pos_f;

	 dist= qf-qi;

	 dirn = 1;
	 if(dist < 0)
	 {
		 dist = - dist;
		 dirn = -1;
	 }


	 d_cruise=.7*dist;
	 t_cruise = (d_cruise)/vi;
	 tb = (dist - d_cruise)/vi;
	 tf = 2*tb + t_cruise;

	 if(dirn == -1)
	 {
	    vi = -vi;
	 }


	 ai = qi;
	 bi = qid;
	 ci = (vi - qid)/(tb+tb);
	 di = (qi+qf-vi*tf)/2;
	 ei = qf;
	 fi = qfd;
	 gi = -ci;

	 samp = tf/2.0e-3;
	 t_int = tf/samp;
	 return (int)  round((samp));
}

int mot_q(int i)
{

    ts =t_int*i;

	  	  if ( ts < tb)
	      {
		          q= ai + ts*bi + ci*ts*ts;
	      }
	      else if (tb <= ts && ts < tf-tb)
	      {
		          q = di+vi*ts;
	      }
	      else if ( tf-tb <= ts && ts <= tf)
	      {
		          q = ei + (ts-tf)*fi + (ts-tf)*(ts-tf)*gi;
	      }

	  return (int) round(q*1000);
}
int mot_qd(int i)
{
	 ts =t_int*i;
	  	  if ( ts < tb)
	      {
	              qd= bi + 2*ts*ci;
	      }
	      else if (tb <= ts && ts < tf-tb)
	      {
	            qd = vi;
	      }
	      else if ( tf-tb <= ts && ts <= tf)
	      {
	              qd = fi + 2*(ts-tf)*gi;
	      }

	  return (int) round(qd);

}
int mot_q2d(int i)
{
	 ts =t_int*i;

	  	  if ( ts < tb)
	      {
		      q2d=2*ci;
	      }
	      else if (tb <= ts && ts < tf-tb)
	      {
		        q2d=0;
	      }
	      else if ( tf-tb <= ts && ts <= tf)
	      {
		        q2d=   2*gi;
	      }
	  return (int) q2d;

}

