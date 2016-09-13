/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>

int sign_function(float a)
{
    if (a < 0)
        return -1;
    else
        return 1;
}


void pid_init(PIDparam &param)
{
    param.Kp = 0;
    param.Ki = 0;
    param.Kd = 0;
    param.integral_limit = 0;
    param.integral = 0;
    param.actual_value_1n = 0;
    param.T_s = 0;
}

void pid_set_parameters(float Kp, float Ki, float Kd, float integral_limit, int T_s, PIDparam &param)
{
    param.Kp = Kp;
    param.Ki = Ki;
    param.Kd = Kd;
    param.integral_limit = integral_limit;
    param.T_s = T_s;
}


float pid_update(float desired_value, float actual_value, int T_s, PIDparam &param)
{
    float error, cmd, integral_term;
    error = desired_value - actual_value;
    param.integral += error;
    integral_term = param.Ki * param.integral;
    if ((integral_term > param.integral_limit) || (integral_term < -param.integral_limit))
        param.integral -= error;
    cmd = ((param.Kp * error) + integral_term - (param.Kd * (actual_value - param.actual_value_1n)));
    param.actual_value_1n = actual_value;
    return cmd;
}


void pid_reset(PIDparam &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
}



float new_pos_controller_updat(float desired_value, float actual_value, int T_s, PIDparam &param)
{
    float cmd, temp;
    temp = param.integral;
    param.integral += ((desired_value * param.Ki) - (actual_value * (param.Ki + param.Kp)) + (param.actual_value_1n * param.Kp));
    if((param.integral > param.integral_limit) || (param.integral < -param.integral_limit))
        param.integral = temp;
    cmd = param.integral - (actual_value * param.Kd) + (param.actual_value_1n * param.Kd);
    param.actual_value_1n = actual_value;
    return cmd;
}


float pos_profiler(float pos_target, float pos_k_1n, float pos_k_2n, posProfilerParam pos_profiler_param)
{
    float velocity_k_1n, temp, deceleration_distance, pos_deceleration, pos_k, pos_temp1, pos_temp2;
    int deceleration_flag = 0;


    if (pos_target == pos_k_1n)
        pos_k = pos_target;
    else if (pos_target > pos_k_1n) {
        if (((pos_k_1n-pos_k_2n)==0) && (pos_target < (pos_k_1n+10)))
            pos_k = pos_k_1n; //ignore the command
        else {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * pos_profiler_param.a_max);
            pos_deceleration = pos_target - deceleration_distance;
            if ((pos_k_1n >= pos_deceleration) && (pos_k_1n > pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * pos_profiler_param.a_max;
            if (deceleration_flag == 0) {
                pos_temp1 = temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = (pos_profiler_param.delta_T * pos_profiler_param.v_max) + pos_k_1n;
                if (pos_temp1 < pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else {
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
        else {
            velocity_k_1n = ((pos_k_1n - pos_k_2n) / pos_profiler_param.delta_T);
            deceleration_distance = (velocity_k_1n * velocity_k_1n) / (2 * pos_profiler_param.a_max);
            pos_deceleration = pos_target + deceleration_distance;
            if ((pos_k_1n <= pos_deceleration) && (pos_k_1n < pos_k_2n))
                deceleration_flag = 1;
            temp = pos_profiler_param.delta_T * pos_profiler_param.delta_T * pos_profiler_param.a_max;
            if (deceleration_flag == 0) {
                pos_temp1 = -temp + (2 * pos_k_1n) - pos_k_2n;
                pos_temp2 = -(pos_profiler_param.delta_T * pos_profiler_param.v_max) + pos_k_1n;
                if (pos_temp1 > pos_temp2)
                    pos_k = pos_temp1;
                else
                    pos_k = pos_temp2;
            }
            else {
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


