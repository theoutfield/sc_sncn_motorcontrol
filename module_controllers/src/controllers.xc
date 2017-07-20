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
 * @brief intializing the parameters of the PIDT1 controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_init(PIDT1param &param)
{
    param.Kp = 0;
    param.Ki = 0;
    param.Kd = 0;
    param.integral_limit = 0;
    param.integral = 0;
    param.derivative = 0;
    param.derivative_1n = 0;
    param.actual_value_1n = 0;
    param.error_value_1n = 0;
    param.T_s = 0;
    param.v = 0;
    param.b = 0;
}

/**
 * @brief setting the parameters of the PIDT1 controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param structure including the parameters of the PIDT1 controller
 *
 * @return void
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDT1param &param)
{
    param.Kp = Kp;
    param.Ki = Ki;
    param.Kd = Kd;
    param.integral_limit = integral_limit;

    if(param.Ki == 0)
        param.integral=0;   //reset the integrator to 0 in case ki is set to 0

    if (param.integral >  param.integral_limit )
        param.integral = param.integral_limit;
    if (param.integral <(-param.integral_limit))
        param.integral =-param.integral_limit;

    if (param.Kd == 0)
        param.derivative = 0; // reset the derivative in case kd is set to 0
    else
        param.v = 13;    // it is recommended to be in range [4, 20]

    param.T_s = T_s;
    param.b = PSEUDO_DERIVATIVE;
}

/**
 * @brief updating the PIDT1 controller.
 * @param desired_value, the reference set point
 * @param actual_value, the actual value (measurement)
 * @param T_s, sampling time
 * @param param, the structure containing the PIDT1 controller parameters
 * @param b, set-point weight, i.e. error = b*y_ref - y
 *
 *
 * @return the output of PIDT1 controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDT1param &param)
{
    double error=0.00, cmd=0.00, derivat_input = 0.00;

    error = desired_value - actual_value;

    /*
     * calculating I part
     */
    param.integral += (param.Ki/2/1000000.00) * (error - param.error_value_1n);

    if ((param.integral >= param.integral_limit) || (param.integral <= -param.integral_limit))
        param.integral -= ((param.Ki/1000000.00) * error);

    if (param.b == 1)
    {
        derivat_input = error - param.error_value_1n;
    }
    else if (param.b == 0)
    {
        derivat_input = -(actual_value-param.actual_value_1n);
    }

    /*
     * calculating D part
     * pseudo derivative controller PDT, i.e. acting on the output of the system
     */
    param.derivative = (((2-T_s*param.v)*param.derivative_1n) + ((2*param.Kd/1000000.00*param.v)*derivat_input)) / (2+T_s*param.v);

    cmd = ((param.Kp/1000000.00) * error) + param.integral + param.derivative;

    param.actual_value_1n = actual_value;
    param.derivative_1n = param.derivative;
    param.error_value_1n = error;

    return cmd;
}

/**
 * @brief resetting the parameters of the PIDT1 controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDT1param &param)
{
    param.actual_value_1n = 0;
    param.integral = 0;
    param.derivative = 0;
    param.derivative_1n = 0;
    param.error_value_1n = 0;
}

