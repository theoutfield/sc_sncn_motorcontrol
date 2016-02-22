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

void init_position_profiler(ProfilerConfig profile_position_config,
                            interface PositionControlInterface client i_position_control,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss) {

    ControlConfig control_config = i_position_control.get_position_control_config();

    HallConfig hall_config;
    QEIConfig qei_config;
    BISSConfig biss_config;

    if (!isnull(i_hall)) {
        hall_config = i_hall.get_hall_config();
    }

    if (!isnull(i_qei)) {
        qei_config = i_qei.get_qei_config();
    }

    if (!isnull(i_biss)) {
        biss_config = i_biss.get_biss_config();
    }

    if(profile_position_config.max_acceleration <= 0 ||
            profile_position_config.max_velocity <= 0){
        printstrln("profile_position: ERROR: Wrong configuration provided to profiler");
        return;
    }

    init_position_profile_limits(profile_position_config.max_acceleration,
                                 profile_position_config.max_velocity,
                                 qei_config, hall_config, biss_config, control_config.feedback_sensor,
                                 profile_position_config.max_position,
                                 profile_position_config.min_position);
}

void set_profile_position(int target_position, int velocity, int acceleration, int deceleration,
                          interface PositionControlInterface client i_position_control )
{
    int i;
    timer t;
    unsigned int time;
    int steps;
    int position_ramp;

    int actual_position = 0;
    int init_state = i_position_control.check_busy();


    if (init_state == INIT_BUSY)
    {
        init_position_control(i_position_control);
    }

    actual_position = i_position_control.get_position();
    steps = init_position_profile(target_position, actual_position, velocity, acceleration, deceleration);
    t :> time;
    for(i = 1; i < steps; i++)
    {
        position_ramp = position_profile_generate(i);
        i_position_control.set_position(position_ramp);
        t when timerafter(time + MSEC_STD) :> time;
    }
    t when timerafter(time + 30 * MSEC_STD) :> time;

}
