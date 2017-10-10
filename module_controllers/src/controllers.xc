/**
 * @file controllers.xc
 * @brief Controllers Libraries
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <controllers.h>
#include <motion_control_service.h>
#include <math.h>
#include <stdlib.h>
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
 * @param input, P parameter scaled by 1e6
 * @param input, I parameter scaled by 1e10
 * @param input, D parameter scaled by 1e6
 * @param input, Integral limit
 * @param input, sampling time in us (microseconds)
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
 * @param T_s, sampling time in us (microseconds)
 * @param param, the structure containing the PIDT1 controller parameters
 *
 * @return the output of PIDT1 controller
 */
double pid_update(double desired_value, double actual_value, double T_s, PIDT1param &param)
{
    double error=0.00, cmd=0.00, derivat_input = 0.00;

    error = desired_value - actual_value;

    /*
     * calculating I part
     */
    param.integral += (param.Ki*T_s/2.0) * (error + param.error_value_1n);

    if ((param.integral >= param.integral_limit) || (param.integral <= -param.integral_limit))
        param.integral -= ((param.Ki*T_s/2.0) * (error + param.error_value_1n));

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
    param.derivative = (((2-T_s*param.v)*param.derivative_1n) + ((2.0*param.Kd*param.v)*derivat_input)) / (2+T_s*param.v);

    cmd = (param.Kp * error) + param.integral + param.derivative;

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

/**
 * @brief initializing params of gains scheduling controller
 * @param the parameters of the controller
 *
 * @return void
 */
void gain_scheduling_init(GSCparam &param)
{
    param.velocity_lo_l = 0;
    param.velocity_hi_l = 0;
    param.pos_Kp_l = 0;
    param.pos_Ki_l = 0;
    param.pos_Kd_l = 0;
    param.pos_Kp_h = 0;
    param.pos_Ki_h = 0;
    param.pos_Kd_h = 0;
    param.vel_Kp_l = 0;
    param.vel_Ki_l = 0;
    param.vel_Kd_l = 0;
    param.vel_Kp_h = 0;
    param.vel_Ki_h = 0;
    param.vel_Kd_h = 0;
}

/**
 * @brief setting params of gains scheduling controller
 * @param position controller P constant for low velocities
 * @param position controller I constant for low velocities
 * @param position controller D constant for low velocitiess
 * @param position controller P constant for high velocities
 * @param position controller I constant for high velocities
 * @param position controller D constant for high velocities
 * @param velocity controller P constant for low velocities
 * @param velocity controller I constant for low velocities
 * @param velocity controller D constant for low velocitiess
 * @param velocity controller P constant for high velocities
 * @param velocity controller I constant for high velocities
 * @param velocity controller D constant for high velocities
 * @param the parameters of the controller
 *
 * @return void
 */
void gain_scheduling_set_param(double pos_Kp_l, double pos_Ki_l, double pos_Kd_l, double pos_Kp_h, double pos_Ki_h, double pos_Kd_h,
        double vel_Kp_l, double vel_Ki_l, double vel_Kd_l, double vel_Kp_h, double vel_Ki_h, double vel_Kd_h,
        int velocity_lo_lim, int velocity_hi_lim, GSCparam &param)
{
    param.pos_Kp_l      = pos_Kp_l;
    param.pos_Ki_l      = pos_Ki_l;
    param.pos_Kd_l      = pos_Kd_l;
    param.pos_Kp_h      = pos_Kp_h;
    param.pos_Ki_h      = pos_Ki_h;
    param.pos_Kd_h      = pos_Kd_h;
    param.vel_Kp_l      = vel_Kp_l;
    param.vel_Ki_l      = vel_Ki_l;
    param.vel_Kd_l      = vel_Kd_l;
    param.vel_Kp_h      = vel_Kp_h;
    param.vel_Ki_h      = vel_Ki_h;
    param.vel_Kd_h      = vel_Kd_h;
    param.velocity_lo_l = velocity_lo_lim;
    param.velocity_hi_l = velocity_hi_lim;
}


/**
 * @brief adjusting gains of controller based on scheduling variable (velocity)
 * @param velocity
 * @param GS controller
 * @param motion control configuration structure
 *
 * @return void
 */

void gain_scheduling_update(int velocity, GSCparam &param_gsc, MotionControlConfig &motion_ctrl_config)
{
    if (abs(velocity) < (double)param_gsc.velocity_lo_l)
    {
        // gain scheduling controller in a low velocity operating point
        motion_ctrl_config.position_kp = param_gsc.pos_Kp_l;
        motion_ctrl_config.position_ki = param_gsc.pos_Ki_l;
        motion_ctrl_config.position_kd = param_gsc.pos_Kd_l;

        motion_ctrl_config.velocity_kp = param_gsc.vel_Kp_l;
        motion_ctrl_config.velocity_ki = param_gsc.vel_Ki_l;
        motion_ctrl_config.velocity_kd = param_gsc.vel_Kd_l;
    }
    else if (abs(velocity) > (double)param_gsc.velocity_hi_l)
    {
        // gain scheduling controller in a high velocity operating point
        motion_ctrl_config.position_kp = param_gsc.pos_Kp_h;
        motion_ctrl_config.position_ki = param_gsc.pos_Ki_h;
        motion_ctrl_config.position_kd = param_gsc.pos_Kd_h;

        motion_ctrl_config.velocity_kp = param_gsc.vel_Kp_h;
        motion_ctrl_config.velocity_ki = param_gsc.vel_Ki_h;
        motion_ctrl_config.velocity_kd = param_gsc.vel_Kd_h;
    }
    else
    {
        // position controller gains scheduling by linear interpolation
        motion_ctrl_config.position_kp = ((param_gsc.pos_Kp_h - param_gsc.pos_Kp_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.pos_Kp_l;
        motion_ctrl_config.position_ki = ((param_gsc.pos_Ki_h - param_gsc.pos_Ki_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.pos_Ki_l;
        motion_ctrl_config.position_kd = ((param_gsc.pos_Kd_h - param_gsc.pos_Kd_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.pos_Kd_l;

        // velocity controller gains scheduling by linear interpolation
        motion_ctrl_config.velocity_kp = ((param_gsc.vel_Kp_h - param_gsc.vel_Kp_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.vel_Kp_l;
        motion_ctrl_config.velocity_ki = ((param_gsc.vel_Ki_h - param_gsc.vel_Ki_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.vel_Ki_l;
        motion_ctrl_config.velocity_kd = ((param_gsc.vel_Kd_h - param_gsc.vel_Kd_l)/(param_gsc.velocity_hi_l - param_gsc.velocity_lo_l)) * (velocity - param_gsc.velocity_lo_l) + param_gsc.vel_Kd_l;
    }
}

