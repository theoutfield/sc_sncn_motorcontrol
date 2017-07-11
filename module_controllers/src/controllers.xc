/**
 * @file controllers.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers.h>
#include <motion_control_service.h>
#include <math.h>
#include <xscope.h>



/**
 * @brief intializing the parameters of the PID controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
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

/**
 * @brief setting the parameters of the PID controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param structure including the parameters of the PID controller
 *
 * @return void
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDparam &param)
{
    param.Kp = Kp;
    param.Ki = Ki;
    param.Kd = Kd;
    param.integral_limit = integral_limit;

    if(param.Ki==0) param.integral=0; //reset the integrator to 0 in case ki is set to 0

    if (param.integral >  param.integral_limit ) param.integral = param.integral_limit;
    if (param.integral <(-param.integral_limit)) param.integral =-param.integral_limit;

    param.T_s = T_s;
}

/**
 * @brief updating the PID controller.
 * @param desired_value, the reference set point
 * @param actual_value, the actual value (measurement)
 * @param T_s, sampling time
 * @param param, the structure containing the pid controller parameters
 *
 *
 * @return the output of pid controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDparam &param)
{
    double error=0.00, cmd=0.00;

    error = desired_value - actual_value;

    param.integral += (param.Ki/1000000.00) * error;
    if ((param.integral >= param.integral_limit) || (param.integral <= -param.integral_limit))
        param.integral -= ((param.Ki/1000000.00) * error);

    cmd = ((param.Kp/1000000.00) * (desired_value- actual_value)) + param.integral - ((param.Kd/1000000.00) * (actual_value - param.actual_value_1n));

    param.actual_value_1n = actual_value;

    return cmd;
}

/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDparam &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
}

