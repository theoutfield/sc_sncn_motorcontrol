/**
 * @file controllers.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motion_control_service.h>

typedef enum
{
    PSEUDO_DERIVATIVE,  // pseudo derivative controller, acts on output
    STANDARD            // derivative acts on error
} derivative_feedback;

/**
 * @brief Structure type to set the parameters of the PIDT1 controller.
 */
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double integral_limit;
    double integral;
    double derivative;
    double derivative_1n;
    double actual_value_1n;
    double error_value_1n;
    int T_s;                // sampling-Time in microseconds
    int v;                  // first order element in D part with time constant T_v
                            // T_v = Td / v => v €[4, 20]
    derivative_feedback b;  // derivative feedback, error = b*ref-y
} PIDT1param;

/**
 * @brief intializing the parameters of the PIDT1 controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_init(PIDT1param &param);

/**
 * @brief setting the parameters of the PIDT1 controller.
 * @param input, P parameter
 * @param input, I parameter
 * @param input, D parameter
 * @param input, Integral limit
 * @param input, sample-time in us (microseconds).
 * @param input, first order element in D part
 * @param structure including the parameters of the PIDT1 controller
 *
 * @return void
 */
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDT1param &param);

/**
 * @brief updating the PIDT1 controller.
 * @param desired_value, the reference set point
 * @param actual_value, the actual value (measurement)
 * @param T_s, sampling time
 * @param param, the structure containing the PIDT1 controller parameters
 *
 *
 * @return the output of pid controller
 */
double pid_update(double desired_value, double actual_value, int T_s, PIDT1param &param);

/**
 * @brief resetting the parameters of the PIDT1 controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDT1param &param);

/**
 * @brief Structure type to set the parameters of the Gain scheduling controller.
 */
typedef struct
{
    double pos_Kp_l;
    double pos_Ki_l;
    double pos_Kd_l;
    double pos_Kp_h;
    double pos_Ki_h;
    double pos_Kd_h;
    double vel_Kp_l;
    double vel_Ki_l;
    double vel_Kd_l;
    double vel_Kp_h;
    double vel_Ki_h;
    double vel_Kd_h;
    int velocity_lo_l;
    int velocity_hi_l;
}GSCparam;

/**
 * @brief initializing params of gains scheduling controller
 * @param the parameters of the controller
 *
 * @return void
 */
void gain_scheduling_init(GSCparam &param);

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
        int velocity_lo_lim, int velocity_hi_lim, GSCparam &param);

/**
 * @brief adjusting gains of controller based on scheduling variable (velocity)
 * @param velocity
 * @param GS controller
 * @param motion control configuration structure
 *
 * @return void
 */

void gain_scheduling_update(int velocity, GSCparam &param_gsc, MotionControlConfig &motion_ctrl_config);



