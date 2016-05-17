/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once



/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    int i1_P;
    int i1_I;
    int i1_D;
    int i1_P_error_limit;
    int i1_I_error_limit;
    int i1_integral_limit;
    int i1_cmd_limit;
    int i1_feedback_1n;
    int i1_error_integral;
    int i1_T_s;    //Sampling-Time in microseconds
} PIDparam;


/**
 * @brief intializing the parameters of the PID controller.
 *
 * @param P
 * @param I
 * @param D
 * @param the error going to the P term will be limited to this value
 * @param the error going to the I term will be limited to this value
 * @param the output of the integral will be limited to this value devided by (I+1)
 * @param the output of the controller will be limited to this value
 * @param the parameters of the controller
 */
void PID_init(int i1_P, int i1_I, int i1_D, int i1_P_error_limit, int i1_I_error_limit, int i1_itegral_limit, int i1_cmd_limit, int i1_T_s, PIDparam &param);


/**
 * @brief updating the controller.
 * @time needed to be processed: --us when the 100MHz teil is fully loaded.
 *
 * @param output, the control comand from i2 type
 * @param input, setpoint
 * @param input, measured value
 * @param sample-time in us (microseconds).
 * @param the parameters of the controller
 */
int PID_update(int i1_setpoint, int i1_feedback, int i1_T_s, PIDparam &param);

