/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once


/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;
    float integral;
    float actual_value_1n;
    int T_s;    //Sampling-Time in microseconds
} PIDparam;




/**
 * @brief intializing the parameters of the PID controller.
 * @param the parameters of the controller
 */
void pid_init(PIDparam &param);




void pid_set_parameters(float Kp, float Ki, float Kd, float integral_limit, int T_s, PIDparam &param);


/**
 * @brief updating the controller.
 * @param output, the control comand.
 * @param input, setpoint
 * @param input, measured value
 * @param sample-time in us (microseconds).
 * @param the parameters of the controller
 */
float pid_update(float desired_value, float actual_value, int T_s, PIDparam &param);

void pid_reset(PIDparam &param);


float new_pos_controller_updat(float desired_value, float actual_value, int T_s, PIDparam &param);



