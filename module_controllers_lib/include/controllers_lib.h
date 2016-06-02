/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define int9_DENOMINATOR 127 //0x7f
#define Uint9_DENOMINATOR 255 //0xff
#define INT16_DENOMINATOR 32767 //0x7fff
#define UINT16_DENOMINATOR 65535 //0xffff

/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    int int9_P;
    int int9_I;
    int int9_D;
    int int21_P_error_limit;
    int int21_I_error_limit;
    int int22_integral_limit;
    int int32_cmd_limit;
    int int20_feedback_p_filter_1n;
    int int20_feedback_d_filter_1n;
    int int22_error_integral;
    int int16_T_s;    //Sampling-Time in microseconds
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
void pid_init(int int9_P, int int9_I, int int9_D, int int21_P_error_limit, int int21_I_error_limit,
              int int16_itegral_limit, int int32_cmd_limit, int int16_T_s, PIDparam &param);




void pid_set_coefficients(int int9_P, int int9_I, int int9_D, PIDparam &param);

void pid_set_limits(int int21_P_error_limit, int int21_I_error_limit, int in16_itegral_limit, int int32_cmd_limit, PIDparam &param);

/**
 * @brief updating the controller.
 * @time needed to be processed: --us when the 100MHz teil is fully loaded.
 *
 * @param output, the control comand. For the number of bits have a look at the note below:
 * @param input, setpoint
 * @param input, measured value
 * @param sample-time in us (microseconds).
 * @param the parameters of the controller
 * NOTE: If the biggest input is int20 and the bigest PID parameter is int9, then the output is int(20+9+3)=int32
 * NOTE: If the biggest input is int15 and the bigest PID parameter is int8, then the output is int(15+8+3)=int26
 */
int pid_update(int int20_setpoint, int int20_feedback_p_filter, int int20_feedback_d_filter, int int16_T_s, PIDparam &param);






