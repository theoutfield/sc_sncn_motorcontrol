
#include "pos_ctrl.h"
#include "refclk.h"
#include <xs1.h>
#include "dc_motor_config.h"
signed int Q, Qd;


extern inline int init(int vel, int pos_i, int pos_f);
extern inline int mot_q(int i);
extern inline int mot_qd(int i);
extern inline int mot_q2d(int i);
int hold_load = 1;


int move(int cur_p,int d_pos, int vi, chanend c_commutation, chanend pos_data)
{
		  // max vi  126 forw  126 revr

		  int ep, ev, Kp, Kv, Ki, Po, cur_v=0, dirn=1, dist, sta=0, samp=0;
		  timer t;
		  unsigned ts_scheduled_ctrl;
		  int speed = 0;
		  unsigned i = 0;
		  signed new;
		  int ge = GEAR_RATIO;
		  int enc_div = POLE_PAIRS * GEAR_RATIO;
		  // 2ms samp
		  samp= init(vi, cur_p, d_pos);
		  sta = 0;
		  dist= d_pos - cur_p;
		  dirn = 1;

		  if(dist < 0)
		  {
			  dirn = -1;
		  }



		  if(hold_load==1)
		  {
			  c_commutation <: 2;
			  c_commutation <: 0;
			  t:>ts_scheduled_ctrl;
			  t when timerafter(ts_scheduled_ctrl+596*USEC_FAST) :> void;
		  }

		  i=0;
		  Kp =2450; Kv =57; Ki =1;
		  //Kp = 1000; Kv = 70;
		  while(!sta)
		  {
			  t:>ts_scheduled_ctrl;
			  t when timerafter(ts_scheduled_ctrl+494250) :> void;  //1977 USEC_FAST  in total 1977+23 2000us or 2 ms

				i = i+1;
				Q   =   mot_q(i);
				Qd  =  (mot_qd(i)*60*ge)/360 ;

				ep = Q  - cur_p ;
				ev = Qd - cur_v ;
				//ei= ei+ep;
				Po = (Kp * ep)/1000 + (Kv * ev)/10  ;//+ Q2d[i]/1000;

				if(dirn==1){
				if (Po > 10580)
					Po = 10580 ;
				else if(Po < 0)
					Po = 0 ;
				}
				else{
					if (Po < -11000)
						Po = -11000 ;
					else if(Po > 0)
						Po = 0 ;
				}

			/*	xscope_probe_data(2, cur_p);
				xscope_probe_data(0, Po);
				xscope_probe_data(1, cur_v);
				xscope_probe_data(3, ep);
				xscope_probe_data(4, ev);
				xscope_probe_data(5, Q);
				xscope_probe_data(6, Qd);*/
				c_commutation <: 2;
				c_commutation <: Po;

				pos_data <: 1;
				pos_data :> new;
				cur_p = (new*100)/ (enc_div);   //*1000 (new*300)/3276;
				c_commutation <: 1;
				c_commutation :> speed;
				cur_v = speed/POLE_PAIRS;   //pole pairs


				while(i>=samp-1)
				{
					//i=samp-1;// Q2d[i]=0;
					if(dirn==1)
					{
						if( cur_p >= d_pos*1000 )
						{
							sta=1;
							if(hold_load==1){
								c_commutation <: 3;
								c_commutation <: 20;
							}
							else{
							c_commutation <: 2;
							c_commutation <: 0;
							}
							break;
						}
						//else if(Po> 320 || cur_p < d_pos*1000)
						//	continue;
					}
					else
					{
						if( cur_p <= d_pos*1000 )
						{
							sta=1;
							if(hold_load==1){
								c_commutation <: 3;
								c_commutation <: 20;
							}
							else{
							c_commutation <: 2;
							c_commutation <: 0;
							}
							break;
						}
					}

					sta=1;
					if(hold_load==1){
						c_commutation <: 3;
						c_commutation <: 20;
					}
					else{
						c_commutation <: 2;
						c_commutation <: 0;
					}

					break;
				}
				if(sta==1)
				{
					if(hold_load==1){
						c_commutation <: 3;
						c_commutation <: 20;
					}
					break;
				}
		  }

		  return 0;
}


