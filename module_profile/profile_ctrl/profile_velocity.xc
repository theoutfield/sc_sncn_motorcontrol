/**
 * @file profile_velocity.xc
 * @brief Profile Velocity Control functions
 *      Implements velocity profile control function
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <refclk.h>
#include <xscope.h>
#include <mc_internal_constants.h>
#include <profile.h>
#include <profile_control.h>

#define USE_XSCOPE

#if 0
void init_velocity_profiler(ProfilerConfig profile_velocity_config,
                                interface VelocityControlInterface client i_velocity_control){

    init_velocity_profile_limits(profile_velocity_config.max_velocity,
                                    profile_velocity_config.max_acceleration,
                                    profile_velocity_config.max_deceleration);
    //Interface not used for the moment, probably in the future.

}

void set_profile_velocity(int target_velocity, int acceleration, int deceleration, interface VelocityControlInterface client i_velocity_control)
{
    int actual_velocity;
    timer t;
    unsigned int time;
    int steps = 0;
    int velocity_ramp;
    int i;
    int init_state = i_velocity_control.check_busy();
    if(init_state == INIT_BUSY)
    {
        init_velocity_control(i_velocity_control);
    }

    actual_velocity = i_velocity_control.get_velocity();
    steps = init_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration);
    t :> time;
    for(i = 1; i < steps; i++) {
        velocity_ramp = velocity_profile_generate(i);
        i_velocity_control.set_velocity(velocity_ramp);

        t when timerafter(time + MSEC_STD) :> time;

#ifdef USE_XSCOPE
        actual_velocity = i_velocity_control.get_velocity();
//        xscope_int(0, actual_velocity);
//        xscope_int(1, velocity_ramp);
#endif
    }
    if (target_velocity == 0) {
        i_velocity_control.set_velocity(target_velocity);
    }
    t when timerafter(time + 30 * MSEC_STD) :> time;

}
#endif

/**
 * @brief updating the velocity reference profiler
 *
 * @param   velocity_ref, target velocity
 * @param   velocity_ref_in_k_1n, profiled velocity calculated in one step
 * @param   velocity_actual, actual value of velocity
 * @param   profiler_param, structure containing the profiler parameters
 * @param   velocity_control_loop, the execution cycle of velocity controller (us)
 *
 * @return  profiled velocity calculated for the next step
 */
double velocity_profiler(double velocity_ref, double velocity_ref_in_k_1n, double velocity_actual, ProfilerParam profiler_param, int position_control_loop)
{

    double velocity_step=0.00, velocity_ref_in_k=0.00;
    double velocity_error=0.00;

    if(velocity_ref>velocity_actual)
        velocity_step = (((double)(position_control_loop)) * profiler_param.acceleration_max )/1000000.00; //rpm/s
    else
        velocity_step = (((double)(position_control_loop)) * profiler_param.deceleration_max )/1000000.00; //rpm/s

    if(velocity_step<0) velocity_step=-velocity_step;

    if(velocity_ref_in_k_1n<velocity_ref)
    {
        velocity_ref_in_k = velocity_ref_in_k_1n + velocity_step;
    }
    else if (velocity_ref_in_k_1n>velocity_ref)
    {
        velocity_ref_in_k = velocity_ref_in_k_1n - velocity_step;
    }

    velocity_error = velocity_ref - velocity_ref_in_k_1n;
    if( (-velocity_step)<velocity_error  && velocity_error<velocity_step)
        velocity_ref_in_k = velocity_ref;

    return velocity_ref_in_k;
}
