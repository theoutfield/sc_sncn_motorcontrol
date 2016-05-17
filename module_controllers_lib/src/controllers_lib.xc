/**
 * @file controllers_lib.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers_lib.h>



void PID_init(int i1_P, int i1_I, int i1_D, int i1_P_error_limit, int i1_I_error_limit, int i1_itegral_limit, int i1_cmd_limit, int i1_T_s, PIDparam &param )
{
    param.i1_P = i1_P;
    param.i1_I = i1_I;
    param.i1_D = i1_D;
    param.i1_P_error_limit = i1_P_error_limit;
    param.i1_I_error_limit = i1_I_error_limit;
    param.i1_integral_limit = i1_itegral_limit;
    param.i1_cmd_limit = i1_cmd_limit;
    param.i1_T_s = i1_T_s;    //Sampling-Time in microseconds
    param.i1_feedback_1n = 0;
    param.i1_error_integral = 0;
}


int PID_update(int i1_setpoint, int i1_feedback, PIDparam &param)
{
    int i1_P_error, i1_I_error, i1_derivative, i2_cmd;

    i1_P_error = i1_setpoint - i1_feedback;
    if (i1_P_error > param.i1_P_error_limit)
        i1_P_error = param.i1_P_error_limit;
    else if (i1_P_error < -param.i1_P_error_limit)
        i1_P_error = -param.i1_P_error_limit;

    i1_I_error = i1_P_error;
    if (i1_I_error > param.i1_I_error_limit)
        i1_I_error = param.i1_I_error_limit;
    else if (i1_I_error < -param.i1_I_error_limit)
        i1_I_error = -param.i1_I_error_limit;

    param.i1_error_integral += i1_I_error;
    if (param.i1_error_integral > param.i1_integral_limit)
        param.i1_error_integral = param.i1_integral_limit;
    else if (param.i1_error_integral < -param.i1_integral_limit)
        param.i1_error_integral = -param.i1_integral_limit;

    i1_derivative = i1_feedback - param.i1_feedback_1n;

    i2_cmd = (param.i1_P * i1_P_error) + (param.i1_I * param.i1_error_integral) - (param.i1_D * i1_derivative);

    param.i1_feedback_1n = i1_derivative;
}


















