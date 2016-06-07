/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define INT9_MIN  -255
#define INT9_MAX   255
#define INT11_MIN -1023
#define INT11_MAX  1023
#define INT16_MIN -32767
#define INT16_MAX  32767
#define INT21_MIN -1048575
#define INT21_MAX  1048575
#define INT23_MIN -4194303
#define INT23_MAX  4194303
#define INT31_MIN -1073741823
#define INT31_MAX  1073741823



/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    int int9_P;
    int int9_I;
    int int9_D;
    int int23_P_error_limit;
    int int23_I_error_limit;
    int int23_integral_limit;
    int int23_cmd_limit;
    int int23_feedback_p_filter_1n;
    int int23_feedback_d_filter_1n;
    int int23_error_integral;
    int int16_T_s;    //Sampling-Time in microseconds
    int scale_factor;
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
void pid_init(int int9_P, int int9_I, int int9_D, int int23_P_error_limit, int int23_I_error_limit,
              int int23_itegral_limit, int int23_cmd_limit, int int16_T_s, PIDparam &param);




void pid_set_coefficients(int int9_P, int int9_I, int int9_D, PIDparam &param);

void pid_set_limits(int int23_P_error_limit, int int23_I_error_limit, int in23_itegral_limit, int int23_cmd_limit, PIDparam &param);

/**
 * @brief updating the controller.
 * @time needed to be processed: --us when the 100MHz teil is fully loaded.
 *
 * @param output, the control comand. For the number of bits have a look at the note below:
 * @param input, setpoint
 * @param input, measured value
 * @param sample-time in us (microseconds).
 * @param the parameters of the controller
 * NOTE: If the biggest input is int23 and the bigest PID parameter is int9, then the output is int((23+9-1)=int31
 * NOTE: If the biggest input is int15 and the bigest PID parameter is int8, then the output is int(15+8-1)=int22
 */
int pid_update(int int23_setpoint, int int23_feedback_p_filter, int int23_feedback_d_filter, int int16_T_s, PIDparam &param);

void pid_reset(PIDparam &param);






