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

/**
 * @brief sign function.
 * @param output, sign of the number
 * @param input, number
 */
int sign_function(float a)
{
    if (a < 0)
        return -1;
    else
        return 1;
}

/**
 * @brief updating the position reference profiler
 * @param   pos_target, target position
 * @param   pos_k_1n, profiled position calculated in one step ago
 * @param   pos_k_2n, profiled position calculated in two steps ago
 * @param   pos_actual, profiled position calculated in three steps ago
 * @param   pos_profiler_param parameters of the position reference profiler
 *
 * @return  profiled position calculated for the next step
 */
float pos_profiler(double pos_target, double pos_k_1n, double pos_k_2n, double pos_actual, ProfilerParam pos_profiler_param)
{
    double velocity_k_1n, temp, deceleration_distance, pos_deceleration, pos_k, pos_temp1, pos_temp2, v_max = 0.00, a_max = 0.00;
    double variable_velocity_distance=0.00;
    int deceleration_flag = 0;

    v_max = (((double)(pos_profiler_param.v_max)) * pos_profiler_param.resolution )/60.00;

    velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);

    if(pos_profiler_param.deceleration_max<pos_profiler_param.acceleration_max)
    {
        a_max = (((double)(pos_profiler_param.deceleration_max)) * pos_profiler_param.resolution )/60.00;
        variable_velocity_distance = (velocity_k_1n*velocity_k_1n) / (4 * a_max);
    }
    else
    {
        a_max = (((double)(pos_profiler_param.acceleration_max)) * pos_profiler_param.resolution )/60.00;
        variable_velocity_distance = (velocity_k_1n*velocity_k_1n) / (4 * a_max);
    }

    if(pos_target>pos_actual  &&  (pos_target-pos_actual)<variable_velocity_distance)
        a_max = (((double)(pos_profiler_param.deceleration_max)) * pos_profiler_param.resolution )/60.00;
    else if(pos_target>pos_actual  &&  (pos_target-pos_actual)>variable_velocity_distance)
        a_max = (((double)(pos_profiler_param.acceleration_max)) * pos_profiler_param.resolution )/60.00;
    else if(pos_target<pos_actual  &&  (pos_actual-pos_target)<variable_velocity_distance)
        a_max = (((double)(pos_profiler_param.deceleration_max)) * pos_profiler_param.resolution )/60.00;
    else if(pos_target<pos_actual  &&  (pos_actual-pos_target)>variable_velocity_distance)
        a_max = (((double)(pos_profiler_param.acceleration_max)) * pos_profiler_param.resolution )/60.00;

    if (pos_target == pos_k_1n)
        pos_k = pos_target;
    else if (pos_target > pos_k_1n)
    {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target < (pos_k_1n+10)))
            pos_k = pos_k_1n; //ignore the command
        else
        {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * a_max);
            pos_deceleration = pos_target - deceleration_distance;
            if ((pos_k_1n >= pos_deceleration) && (pos_k_1n > pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * a_max;
            if (deceleration_flag == 0)
            {
                pos_temp1 = temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = (pos_profiler_param.delta_T * v_max) + pos_k_1n;
                if (pos_temp1 < pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else
            {
                pos_k = -temp + (2 * pos_k_1n) - pos_k_2n;
            }
            if (pos_k > pos_target)
                pos_k = pos_target;
            if ((pos_k < pos_target) && (sign_function(pos_k_1n-pos_k_2n) > sign_function(pos_k-pos_k_1n)))
                pos_k = pos_target;
        }
    }
    else
    {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target > (pos_k_1n-10)))
            pos_k = pos_k_1n; //ignore the command
        else
        {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * a_max);
            pos_deceleration = pos_target + deceleration_distance;
            if ((pos_k_1n <= pos_deceleration) && (pos_k_1n < pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * a_max;
            if (deceleration_flag == 0)
            {
                pos_temp1 = -temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = -(pos_profiler_param.delta_T * v_max) + pos_k_1n;
                if (pos_temp1 > pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else
            {
                pos_k = temp + (2 * pos_k_1n) - pos_k_2n;
            }
            if (pos_k < pos_target)
                pos_k = pos_target;
            if ((pos_k > pos_target) && (sign_function(pos_k_1n-pos_k_2n) < sign_function(pos_k-pos_k_1n)))
                pos_k = pos_target;
        }
    }

    return pos_k;
}
