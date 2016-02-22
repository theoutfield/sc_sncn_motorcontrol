/**
 * @file profile_torque.xc
 * @brief Profile Torque Control functions
 *      Implements torque profile control function
 * @author Synapticon GmbH <support@synapticon.com>
*/


#include <refclk.h>
#include <profile.h>
#include <profile_control.h>

void init_torque_profiler(ProfilerConfig profile_torque_config,
                                interface TorqueControlInterface client i_torque_control){

    init_linear_profile_limits(profile_torque_config.max_current, profile_torque_config.polarity);

    //Interface not used for the moment, likely in the future
}

void set_profile_torque(int target_torque, int torque_slope, interface TorqueControlInterface client i_torque_control)
{
    int i;
    int steps;
    int torque_ramp;
    int actual_torque;
    timer t;
    unsigned int time;

    int init_state = i_torque_control.check_busy();
    if (init_state == INIT_BUSY) {
        init_torque_control(i_torque_control);
    }

    actual_torque = i_torque_control.get_torque() * get_linear_profile_polarity();
    steps = init_linear_profile(target_torque, actual_torque, torque_slope, torque_slope);
    t :> time;
    for(i = 1; i<steps; i++) {
        torque_ramp =  linear_profile_generate(i);
        i_torque_control.set_torque(torque_ramp);
        /*actual_torque = i_torque_control.get_torque() * get_linear_profile_polarity();
          xscope_int(0, actual_torque);
          xscope_int(1, torque_ramp);*/
        t when timerafter(time + MSEC_STD) :> time;
    }
    t when timerafter(time + 30 * MSEC_STD) :> time;
}
