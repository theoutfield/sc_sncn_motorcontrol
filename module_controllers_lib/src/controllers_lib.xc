/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>


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


float pid_update(float desired_value, float actual_value, int feedforward_ctrl_effort, int T_s, PIDparam &param)
{
    float error, cmd, integral_term;

    error = desired_value - actual_value;

    param.integral += error;
    integral_term = param.Ki * param.integral;
    if ((integral_term > param.integral_limit) || (integral_term < -param.integral_limit))
        param.integral -= error;

    cmd = ((param.Kp * error) + integral_term - (param.Kd * (actual_value - param.actual_value_1n)));
    cmd /= param.scale_factor;
    cmd += ((float) feedforward_ctrl_effort);

    param.actual_value_1n = actual_value;

    return cmd;
}


void pid_reset(PIDparam &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
}



float new_pos_controller_updat(float desired_value, float actual_value, float feedforward_ctrl_effort, int T_s, PIDparam &param)
{
    float cmd, temp;

    temp = param.integral;
    param.integral += ((desired_value * param.Ki) - (actual_value * (param.Ki + param.Kp)) + (param.actual_value_1n * param.Kp));
    if((param.integral > param.integral_limit) || (param.integral < -param.integral_limit))
        param.integral = temp;
    cmd = param.integral - (actual_value * param.Kd) + (param.actual_value_1n * param.Kd);
    cmd += feedforward_ctrl_effort;
    param.actual_value_1n = actual_value;
    return cmd;
}





