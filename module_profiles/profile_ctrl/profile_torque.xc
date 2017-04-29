/**
 * @file profile_torque.xc
 * @brief Profile Torque Control functions
 *      Implements torque profile control function
 * @author Synapticon GmbH <support@synapticon.com>
*/


#include <refclk.h>
#include <profile.h>
#include <profile_control.h>

//FIXME
//void init_torque_profiler(ProfilerConfig profile_torque_config,
//                                interface TorqueControlInterface client i_torque_control){
//
//    init_linear_profile_limits(profile_torque_config.max_current, profile_torque_config.polarity);
//
//    //Interface not used for the moment, likely in the future
//}
//
//void set_profile_torque(int target_torque, int torque_slope, interface TorqueControlInterface client i_torque_control)
//{
//    int i;
//    int steps;
//    int torque_ramp;
//    int actual_torque;
//    timer t;
//    unsigned int time;
//
//    int init_state = i_torque_control.check_busy();
//    if (init_state == INIT_BUSY) {
//        init_torque_control(i_torque_control);
//    }
//
//    actual_torque = i_torque_control.get_torque() * get_linear_profile_polarity();
//    steps = init_linear_profile(target_torque, actual_torque, torque_slope, torque_slope);
//    t :> time;
//    for(i = 1; i<steps; i++) {
//        torque_ramp =  linear_profile_generate(i);
//        i_torque_control.set_torque(torque_ramp);
//        /*actual_torque = i_torque_control.get_torque() * get_linear_profile_polarity();
//          xscope_int(0, actual_torque);
//          xscope_int(1, torque_ramp);*/
//        t when timerafter(time + MSEC_STD) :> time;
//    }
//    t when timerafter(time + 30 * MSEC_STD) :> time;
//}


/**
 * @brief updating the torque reference profiler
 *
 * @param   torque_ref, target torque
 * @param   torque_ref_in_k_1n, profiled torque calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   torque_control_loop, the execution cycle of torque controller (us)
 *
 * @return  profiled torque calculated for the next step
 */
double torque_profiler(double torque_ref, double torque_ref_in_k_1n, ProfilerParam profiler_param, int position_control_loop)
{

    double torque_step=0.00, torque_ref_in_k=0.00;
    double torque_error=0.00;

    torque_step = (((double)(position_control_loop)) * profiler_param.torque_rate_max )/1000000.00;
    //r_max [mNm/s] / 1000 [mNm/ms] * (position_control_loop/1000) mNm
    if(torque_step<0) torque_step=-torque_step;

    if(torque_ref_in_k_1n<torque_ref)
    {
        torque_ref_in_k = torque_ref_in_k_1n + torque_step;
    }
    else if (torque_ref_in_k_1n>torque_ref)
    {
        torque_ref_in_k = torque_ref_in_k_1n - torque_step;
    }

    torque_error = torque_ref - torque_ref_in_k_1n;
    if( (-torque_step)<torque_error  && torque_error<torque_step)
        torque_ref_in_k = torque_ref;

    return torque_ref_in_k;
}
