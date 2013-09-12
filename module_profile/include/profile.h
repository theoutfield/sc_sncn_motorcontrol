//#include <dc_motor_config.h>
#include <stdio.h>
#include <math.h>

#ifndef _PROFILE_H_
#define _PROFILE_H_
/*Profile Velocity Quick Stop*/

extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);

extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/

extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration, int max_velocity);

extern int velocity_profile_generate(int step);

/*Profile Position Mode*/

extern void init_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity);

extern int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
        						 int deceleration);

extern int position_profile_generate(int step);

/*Profile Position Quick Stop*/

extern int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration);  		//emergency stop profile

extern  int quick_stop_position_profile_generate(int steps, int actual_velocity);

#endif /* _PROFILE_H_ */
