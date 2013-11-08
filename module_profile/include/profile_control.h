


#ifndef _PROFILE_CONTROL_H_
#define _PROFILE_CONTROL_H_


#include <xs1.h>
#include <platform.h>
#include <dc_motor_config.h>

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration, chanend c_position_ctrl);
void set_profile_velocity(int target_velocity, int acceleration, int deceleration, int max_profile_velocity, chanend c_velocity_ctrl);
void set_profile_torque(int target_torque, int torque_slope, cst_par &cst_params, chanend c_torque_ctrl);

#endif /* _PROFILE_CONTROL_H_ */
