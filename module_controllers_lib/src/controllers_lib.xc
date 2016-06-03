/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>


void pid_init(int int9_P, int int9_I, int int9_D, int int21_P_error_limit, int int21_I_error_limit,
              int int22_integral_limit, int int32_cmd_limit, int int16_T_s, PIDparam &param)
{
    param.int9_P = int9_P;
    param.int9_I = int9_I;
    param.int9_D = int9_D;
    param.int21_P_error_limit = int21_P_error_limit;
    param.int21_I_error_limit = int21_I_error_limit;
    param.int22_integral_limit = int22_integral_limit;
    param.int32_cmd_limit = int32_cmd_limit;
    param.int16_T_s = int16_T_s;    //Sampling-Time in microseconds
    param.int20_feedback_p_filter_1n = 0;
    param.int20_feedback_d_filter_1n = 0;
    param.int22_error_integral = 0;
}

void pid_set_coefficients(int int9_P, int int9_I, int int9_D, PIDparam &param)
{
    param.int9_P = int9_P;
    param.int9_I = int9_I;
    param.int9_D = int9_D;
}

void pid_set_limits(int int21_P_error_limit, int int21_I_error_limit, int int22_integral_limit, int int32_cmd_limit, PIDparam &param)
{
    param.int21_P_error_limit = int21_P_error_limit;
    param.int21_I_error_limit = int21_I_error_limit;
    param.int22_integral_limit = int22_integral_limit;
    param.int32_cmd_limit = int32_cmd_limit;
}

int pid_update(int int20_setpoint, int int20_feedback_p_filter, int int20_feedback_d_filter, int int16_T_s, PIDparam &param)
{
    int int21_P_error, int21_I_error, int21_derivative, int32_cmd;

    int21_P_error = int20_setpoint - int20_feedback_p_filter;
    if (int21_P_error > param.int21_P_error_limit)
        int21_P_error = param.int21_P_error_limit;
    else if (int21_P_error < -param.int21_P_error_limit)
        int21_P_error = -param.int21_P_error_limit;

    int21_I_error = int21_P_error;
    if (int21_I_error > param.int21_I_error_limit)
        int21_I_error = param.int21_I_error_limit;
    else if (int21_I_error < -param.int21_I_error_limit)
        int21_I_error = -param.int21_I_error_limit;

    param.int22_error_integral += int21_I_error;
    if (param.int22_error_integral > param.int22_integral_limit)
        param.int22_error_integral = param.int22_integral_limit;
    else if (param.int22_error_integral < -param.int22_integral_limit)
        param.int22_error_integral = -param.int22_integral_limit;

    int21_derivative = int20_feedback_d_filter - param.int20_feedback_d_filter_1n;

    int32_cmd = ((param.int9_P * int21_P_error) + (param.int9_I * param.int22_error_integral) - (param.int9_D * int21_derivative));
    if (int32_cmd > param.int32_cmd_limit)
        int32_cmd = param.int32_cmd_limit;
    else if (int32_cmd < -param.int32_cmd_limit)
        int32_cmd = -param.int32_cmd_limit;

    param.int20_feedback_p_filter_1n = int20_feedback_p_filter;
    param.int20_feedback_d_filter_1n = int20_feedback_d_filter;

    return int32_cmd;
}
