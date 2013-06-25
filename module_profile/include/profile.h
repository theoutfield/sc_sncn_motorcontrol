#include "refclk.h"

/*Profile Velocity Quick Stop*/
extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);
extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/
extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration);
extern int velocity_profile_generate(int step);

/*Profile Position Mode*/
extern int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
        						 int deceleration);
extern int position_profile_generate(int step);
