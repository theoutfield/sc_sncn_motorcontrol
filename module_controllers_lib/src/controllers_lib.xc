/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>


void pid_init(int int8_P, int int8_I, int int8_D, int int16_P_error_limit, int int16_I_error_limit,
              int int16_itegral_limit, int int16_cmd_limit, int int16_T_s, PIDparam &param)
{
    param.int8_P = int8_P;
    param.int8_I = int8_I;
    param.int8_D = int8_D;
    param.int16_P_error_limit = int16_P_error_limit;
    param.int16_I_error_limit = int16_I_error_limit;
    param.int16_integral_limit = int16_itegral_limit;
    param.int16_cmd_limit = int16_cmd_limit;
    param.int16_T_s = int16_T_s;    //Sampling-Time in microseconds
    param.int16_feedback_p_filter_1n = 0;
    param.int16_feedback_d_filter_1n = 0;
    param.int16_error_integral = 0;
}

void pid_set_coefficients(int int8_P, int int8_I, int int8_D, PIDparam &param)
{
    param.int8_P = int8_P;
    param.int8_I = int8_I;
    param.int8_D = int8_D;
}

void pid_set_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit, PIDparam &param)
{
    param.int16_P_error_limit = int16_P_error_limit;
    param.int16_I_error_limit = int16_I_error_limit;
    param.int16_integral_limit = int16_itegral_limit;
    param.int16_cmd_limit = int16_cmd_limit;
}

int pid_update(int int16_setpoint, int int16_feedback_p_filter, int int16_feedback_d_filter, int int16_T_s, PIDparam &param)
{
    int int16_P_error, int16_I_error, int16_derivative, int16_cmd;

    int16_P_error = int16_setpoint - int16_feedback_p_filter;
    if (int16_P_error > param.int16_P_error_limit)
        int16_P_error = param.int16_P_error_limit;
    else if (int16_P_error < -param.int16_P_error_limit)
        int16_P_error = -param.int16_P_error_limit;

    int16_I_error = int16_P_error;
    if (int16_I_error > param.int16_I_error_limit)
        int16_I_error = param.int16_I_error_limit;
    else if (int16_I_error < -param.int16_I_error_limit)
        int16_I_error = -param.int16_I_error_limit;

    param.int16_error_integral += int16_I_error;
    if (param.int16_error_integral > param.int16_integral_limit)
        param.int16_error_integral = param.int16_integral_limit;
    else if (param.int16_error_integral < -param.int16_integral_limit)
        param.int16_error_integral = -param.int16_integral_limit;

    int16_derivative = int16_feedback_d_filter - param.int16_feedback_d_filter_1n;

    int16_cmd = (((param.int8_P * int16_P_error) + (param.int8_I * param.int16_error_integral) - (param.int8_D * int16_derivative)) / INT8_DENOMINATOR);
    if (int16_cmd > param.int16_cmd_limit)
        int16_cmd = param.int16_cmd_limit;
    else if (int16_cmd < -param.int16_cmd_limit)
        int16_cmd = -param.int16_cmd_limit;

    param.int16_feedback_p_filter_1n = int16_feedback_p_filter;
    param.int16_feedback_d_filter_1n = int16_feedback_d_filter;

    return int16_cmd;
}
