/**
 * @file controllers.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motion_control_service.h>

/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    double Kp;
    double Ki;
    double Kd;
    double integral_limit;
    double integral;
    double actual_value_1n;
    int T_s;    //Sampling-Time in microseconds
} PIDparam;

/**
 * @brief intializing the parameters of the PID controller.
 *
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_init(PIDparam &param);

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
void pid_set_parameters(double Kp, double Ki, double Kd, double integral_limit, int T_s, PIDparam &param);

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
double pid_update(double desired_value, double actual_value, int T_s, PIDparam &param);

/**
 * @brief resetting the parameters of the PID controller.
 * @param the parameters of the controller
 *
 * @return void
 */
void pid_reset(PIDparam &param);




