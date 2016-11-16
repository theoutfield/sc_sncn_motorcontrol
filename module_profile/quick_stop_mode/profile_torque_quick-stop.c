/**
 * @file profile_torque_quick-stop.c
 * @brief Quick stop Profile Generation for Torque
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <profile.h>

struct
{
    int steps;
} qstop_torque_params;

int init_quick_stop_torque_profile(int actual_velocity, int quick_stop_ramp)
{
    /* FIXME this stub only has one step */
    return (int)1;
}


int quick_stop_torque_profile_generate(int step)
{
    /* FIXME apply 0 torque */
    return (int)0;
}
