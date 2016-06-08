/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>


void pid_init(int int9_P, int int9_I, int int9_D, int int23_P_error_limit, int int23_I_error_limit,
              int int23_integral_limit, int int23_cmd_limit, int int16_T_s, PIDparam &param)
{
    param.int9_P = int9_P;
    if (param.int9_P > INT9_MAX)
        param.int9_P = INT9_MAX;
    else if (param.int9_P < INT9_MIN)
        param.int9_P = INT9_MIN;

    param.int9_I = int9_I;
    if (param.int9_I > INT9_MAX)
        param.int9_I = INT9_MAX;
    else if (param.int9_I < INT9_MIN)
        param.int9_I = INT9_MIN;

    param.int9_D = int9_D;
    if (param.int9_D > INT9_MAX)
        param.int9_D = INT9_MAX;
    else if (param.int9_D < INT9_MIN)
        param.int9_D = INT9_MIN;

    param.int23_P_error_limit = int23_P_error_limit;
    if (param.int23_P_error_limit > INT23_MAX)
        param.int23_P_error_limit = INT23_MAX;
    else if (param.int23_P_error_limit < INT23_MIN)
        param.int23_P_error_limit = INT23_MIN;

    param.int23_I_error_limit = int23_I_error_limit;
    if (param.int23_I_error_limit > INT23_MAX)
        param.int23_I_error_limit = INT23_MAX;
    else if (param.int23_I_error_limit < INT23_MIN)
        param.int23_I_error_limit = INT23_MIN;

    param.int23_integral_limit = int23_integral_limit;
    if (param.int23_integral_limit > INT23_MAX)
        param.int23_integral_limit = INT23_MAX;
    else if (param.int23_integral_limit < INT23_MIN)
        param.int23_integral_limit = INT23_MIN;

    param.int23_cmd_limit = int23_cmd_limit;
    if (param.int23_cmd_limit > INT23_MAX)
        param.int23_cmd_limit = INT23_MAX;
    else if (param.int23_cmd_limit < INT23_MIN)
        param.int23_cmd_limit = INT23_MIN;

    param.int16_T_s = int16_T_s;    //Sampling-Time in microseconds
    if (param.int16_T_s > INT16_MAX)
        param.int16_T_s = INT16_MAX;
    else if (param.int16_T_s < INT16_MIN)
        param.int16_T_s = INT16_MIN;

    param.int23_feedback_p_filter_1n = 0;
    param.int23_feedback_d_filter_1n = 0;
    param.int23_error_integral = 0;
}

void pid_set_coefficients(int int9_P, int int9_I, int int9_D, PIDparam &param)
{
    param.int9_P = int9_P;
    if (param.int9_P > INT9_MAX)
        param.int9_P = INT9_MAX;
    else if (param.int9_P < INT9_MIN)
        param.int9_P = INT9_MIN;

    param.int9_I = int9_I;
    if (param.int9_I > INT9_MAX)
        param.int9_I = INT9_MAX;
    else if (param.int9_I < INT9_MIN)
        param.int9_I = INT9_MIN;

    param.int9_D = int9_D;
    if (param.int9_D > INT9_MAX)
        param.int9_D = INT9_MAX;
    else if (param.int9_D < INT9_MIN)
        param.int9_D = INT9_MIN;
}

void pid_set_limits(int int23_P_error_limit, int int23_I_error_limit, int int23_integral_limit, int int23_cmd_limit, PIDparam &param)
{
    param.int23_P_error_limit = int23_P_error_limit;
    if (param.int23_P_error_limit > INT23_MAX)
        param.int23_P_error_limit = INT23_MAX;
    else if (param.int23_P_error_limit < INT23_MIN)
        param.int23_P_error_limit = INT23_MIN;

    param.int23_I_error_limit = int23_I_error_limit;
    if (param.int23_I_error_limit > INT23_MAX)
        param.int23_I_error_limit = INT23_MAX;
    else if (param.int23_I_error_limit < INT23_MIN)
        param.int23_I_error_limit = INT23_MIN;

    param.int23_integral_limit = int23_integral_limit;
    if (param.int23_integral_limit > INT23_MAX)
        param.int23_integral_limit = INT23_MAX;
    else if (param.int23_integral_limit < INT23_MIN)
        param.int23_integral_limit = INT23_MIN;

    param.int23_cmd_limit = int23_cmd_limit;
    if (param.int23_cmd_limit > INT23_MAX)
        param.int23_cmd_limit = INT23_MAX;
    else if (param.int23_cmd_limit < INT23_MIN)
        param.int23_cmd_limit = INT23_MIN;
}

int pid_update(int int23_setpoint, int int23_feedback_p_filter, int int23_feedback_d_filter, int int23_feedforward_ctrl_effort, int int16_T_s, PIDparam &param)
{
    int int23_P_error, int23_I_error, int23_derivative, int31_cmd, int23_cmd;

    int23_P_error = int23_setpoint - int23_feedback_p_filter;
    if (int23_P_error > param.int23_P_error_limit)
        int23_P_error = param.int23_P_error_limit;
    else if (int23_P_error < -param.int23_P_error_limit)
        int23_P_error = -param.int23_P_error_limit;

    int23_I_error = int23_P_error;
    if (int23_I_error > param.int23_I_error_limit)
        int23_I_error = param.int23_I_error_limit;
    else if (int23_I_error < -param.int23_I_error_limit)
        int23_I_error = -param.int23_I_error_limit;

    param.int23_error_integral += int23_I_error;
    if (param.int23_error_integral > param.int23_integral_limit)
        param.int23_error_integral = param.int23_integral_limit;
    else if (param.int23_error_integral < -param.int23_integral_limit)
        param.int23_error_integral = -param.int23_integral_limit;

    int23_derivative = int23_feedback_d_filter - param.int23_feedback_d_filter_1n;
    if (int23_derivative > INT23_MAX)
        int23_derivative = INT23_MAX;
    else if (int23_derivative < INT23_MIN)
        int23_derivative = INT23_MIN;

    int31_cmd = ((param.int9_P * int23_P_error) + (param.int9_I * param.int23_error_integral) - (param.int9_D * int23_derivative));
    int31_cmd /= param.scale_factor;
    int31_cmd += int23_feedforward_ctrl_effort;
    if (int31_cmd > param.int23_cmd_limit)
        int23_cmd = param.int23_cmd_limit;
    else if (int31_cmd < -param.int23_cmd_limit)
        int23_cmd = -param.int23_cmd_limit;
    else
        int23_cmd = int31_cmd;

    param.int23_feedback_p_filter_1n = int23_feedback_p_filter;
    param.int23_feedback_d_filter_1n = int23_feedback_d_filter;

    return int23_cmd;
}


void pid_reset(PIDparam &param)
{
    param.int23_feedback_p_filter_1n = 0;
    param.int23_feedback_d_filter_1n = 0;
    param.int23_error_integral = 0;
}









