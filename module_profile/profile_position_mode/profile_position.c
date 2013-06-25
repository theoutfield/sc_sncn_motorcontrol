//Profile Position Mode
#include "stdbool.h"
#include <math.h>
#include <stdio.h>

struct PROFILE_POSITION_PARAM {
	float max_acceleration;     // max acceleration
	float max_deceleration; 	// max deceleration

	/*User Inputs*/
	float acc;					// deceleration
	float dec; 					// acceleration
	float vi;					// velocity
	float qi;					// initial position
	float qf; 				    // final position

	/*Profile time*/
	float T;    				// total no. of Samples
	int length; 				// total samples in int
	float s_time; 				// sampling time

	int direction;
	int acc_too_low;			// flag for low acceleration constraint

	/*LFPB motion profile constants*/
	float ai;
	float bi;
	float ci;
	float di;
	float ei;
	float fi;
	float gi;

	/*internal velocity variables*/
	float qid;					// initial velocity
	float qfd;					// final velocity

	float distance_cruise;
	float total_distance;
	float distance_acc;			// distance covered during acceleration
	float distance_dec;			// distance covered during deceleration
	float distance_left;		// distance left for cruise velocity

	float tb_acc;				// blend time for acceleration profile
	float tb_dec;				// blend time for deceleration profile
	float tf;					// total time for profile
	float t_cruise;				// time for cruise velocity profile
	float ts;					// variable to hold current sample time

}profile_pos_params;

int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
		                  int deceleration)
{
	profile_pos_params.qf = (float) target_position;

	profile_pos_params.qi = (float) actual_position;

	profile_pos_params.vi = (float) velocity;

	profile_pos_params.acc = (float) acceleration;

	profile_pos_params.dec = (float) deceleration;

	//Internal params
	profile_pos_params.acc_too_low = 0;

	profile_pos_params.qid = 0.0f;

	profile_pos_params.qfd = 0.0f;
	// leads to shorter blend time in begining(if init cond != 0)

	profile_pos_params.total_distance = profile_pos_params.qf - profile_pos_params.qi; // compute distance	flag = 0;

	//printf("total dist %f\n", total_distance);

	profile_pos_params.direction = 1;

	if (profile_pos_params.total_distance < 0)
	{
		profile_pos_params.total_distance = -profile_pos_params.total_distance;

		profile_pos_params.direction = -1;
	}

	profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.acc;

	profile_pos_params.tb_dec = profile_pos_params.vi / profile_pos_params.dec;

	profile_pos_params.distance_acc = (profile_pos_params.acc * profile_pos_params.tb_acc \
			                        * profile_pos_params.tb_acc) / 2.0f;

	profile_pos_params.distance_dec = (profile_pos_params.dec * profile_pos_params.tb_dec \
			                        * profile_pos_params.tb_dec) / 2.0f;

	profile_pos_params.distance_left = profile_pos_params.total_distance \
									 - profile_pos_params.distance_acc   \
									 - profile_pos_params.distance_dec;

	//printf("\n t_acc %f  tb_dec %f \n", t_acc, t_dec);
	//printf("\n distance_acc %f  distance_dec %f distance_left %f \n", distance_acc, distance_dec, distance_left);


	if (profile_pos_params.distance_left < 0)
	{
		profile_pos_params.acc_too_low = 1;
		//printf("acc too low to meet distance/vel constraint\n");
	}
	else if (profile_pos_params.distance_left > 0)
	{
		profile_pos_params.acc_too_low = 0;
	}

	if (profile_pos_params.acc_too_low == 1)
	{
		profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.max_acceleration;

		profile_pos_params.tb_dec = profile_pos_params.tb_acc;

		profile_pos_params.distance_acc = (profile_pos_params.max_acceleration \
				                        * profile_pos_params.tb_acc * profile_pos_params.tb_acc) / 2.0f;

		profile_pos_params.distance_left = profile_pos_params.total_distance - 2.0f \
										 * profile_pos_params.distance_acc;
	}

	if (profile_pos_params.distance_left < 0) {

		profile_pos_params.acc_too_low = 1;
		//printf("not possible\n");
		//printf("acc too low to meet distance/vel constraint\n");  //vel too high for distance constraint

		profile_pos_params.vi = profile_pos_params.total_distance; // reset vi to safe value

		profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.max_acceleration; // last correction possible vi = dist;

		profile_pos_params.tb_dec = profile_pos_params.tb_acc;

		profile_pos_params.distance_acc = (profile_pos_params.max_acceleration \
										* profile_pos_params.tb_acc * profile_pos_params.tb_acc) / 2.0f;

		profile_pos_params.distance_left = profile_pos_params.total_distance - 2.0f \
		                                 * profile_pos_params.distance_acc;
	}
	else if (profile_pos_params.distance_left > 0)
	{
		profile_pos_params.acc_too_low = 0;
	}

//	profile_pos_params.tb_acc = profile_pos_params.t_acc; // blend time for acc

//	profile_pos_params.tb_dec = profile_pos_params.t_dec; // blend time for dec

	profile_pos_params.distance_cruise = profile_pos_params.distance_left;

	profile_pos_params.t_cruise = (profile_pos_params.distance_cruise) / profile_pos_params.vi;

	profile_pos_params.tf = profile_pos_params.tb_acc + profile_pos_params.tb_dec \
			              + profile_pos_params.t_cruise;

	//printf("\n tb_acc %f  tb_dec %f  tf %f\n", tb_acc, tb_dec, tf);

	if (profile_pos_params.direction == -1)
	{
		profile_pos_params.vi = -profile_pos_params.vi;
	}

	/*motion constants*/

	profile_pos_params.ai = profile_pos_params.qi;

	profile_pos_params.bi = profile_pos_params.qid;

	profile_pos_params.ci = (profile_pos_params.vi - profile_pos_params.qid) / (2.0f * profile_pos_params.tb_acc);

	profile_pos_params.di = profile_pos_params.ai + profile_pos_params.tb_acc * profile_pos_params.bi \
						  + profile_pos_params.ci * profile_pos_params.tb_acc * profile_pos_params.tb_acc \
						  - profile_pos_params.vi * profile_pos_params.tb_acc;

	profile_pos_params.ei = profile_pos_params.qf;

	profile_pos_params.fi = profile_pos_params.qfd;

	profile_pos_params.gi = (profile_pos_params.di + (profile_pos_params.tf - profile_pos_params.tb_dec) \
			 	 	 	  * profile_pos_params.vi + profile_pos_params.fi * profile_pos_params.tb_dec \
			              - profile_pos_params.ei) / (profile_pos_params.tb_dec * profile_pos_params.tb_dec);

	profile_pos_params.T = profile_pos_params.tf / 1e-3;           // 1 KHz

	profile_pos_params.length = (int) round(profile_pos_params.T);

	profile_pos_params.s_time = 0.001;								// 1 KHz

	return profile_pos_params.length;
}

int position_profile_generate(int step)
{
	profile_pos_params.ts = profile_pos_params.s_time * step ;

	if (profile_pos_params.ts < profile_pos_params.tb_acc)
	{
		return profile_pos_params.ai + profile_pos_params.ts * profile_pos_params.bi \
			   + profile_pos_params.ci * profile_pos_params.ts * profile_pos_params.ts;
	}

	else if (profile_pos_params.tb_acc <= profile_pos_params.ts \
			 &&  profile_pos_params.ts < (profile_pos_params.tf - profile_pos_params.tb_dec) )
	{
		return profile_pos_params.di + profile_pos_params.vi * profile_pos_params.ts;
	}

	else if ( (profile_pos_params.tf - profile_pos_params.tb_dec) <= profile_pos_params.ts \
			                             && profile_pos_params.ts <= profile_pos_params.tf)
	{
		return profile_pos_params.ei + (profile_pos_params.ts - profile_pos_params.tf) \
			 * profile_pos_params.fi + (profile_pos_params.ts - profile_pos_params.tf) \
			 * (profile_pos_params.ts - profile_pos_params.tf) * profile_pos_params.gi;
	}
}
