/**
 * @file profile_position.xc
 * @brief Profile Position Control functions
 *      Implements position profile control function
 * @author Synapticon GmbH <support@synapticon.com>
*/
#include <refclk.h>
#include <print.h>
#include <profile.h>
#include <profile_control.h>
#include <xs1.h>

void init_position_profiler(ProfilerConfig profile_position_config) {

    if(profile_position_config.max_acceleration <= 0 ||
            profile_position_config.max_velocity <= 0){
        printstrln("profile_position: ERROR: Wrong configuration provided to profiler");
        return;
    }

    init_position_profile_limits(profile_position_config.max_acceleration,
                                 profile_position_config.max_velocity,
                                 profile_position_config.max_position,
                                 profile_position_config.min_position,
                                 profile_position_config.ticks_per_turn);
}

void set_profile_position(DownstreamControlData &downstream_control_data, int velocity, int acceleration, int deceleration,
                          interface PositionVelocityCtrlInterface client i_position_control )
{
    int i;
    timer t;
    unsigned int time;
    int steps;
    int position_ramp;

    int actual_position = 0;
    //FIXME check the state of the position control service
//    int init_state = i_position_control.check_busy();
//
//
//    if (init_state == INIT_BUSY)
//    {
//        init_position_velocity_control(i_position_control);
//    }
    i_position_control.enable_position_ctrl(POS_PID_CONTROLLER);

    actual_position = i_position_control.get_position();

    steps = init_position_profile(downstream_control_data.position_cmd, actual_position, velocity, acceleration, deceleration);

    t :> time;
    for(i = 1; i < steps; i++)
    {
        position_ramp = position_profile_generate(i);
        downstream_control_data.position_cmd = position_ramp;
        i_position_control.update_control_data(downstream_control_data);
        t when timerafter(time + MSEC_STD) :> time;
    }
    t when timerafter(time + 30 * MSEC_STD) :> time;

}
