/**
 * @file controllers_lib.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define INT8_DENOMINATOR 127 //0x7f
#define UINT8_DENOMINATOR 255 //0xff
#define INT16_DENOMINATOR 32767 //0x7fff
#define UINT16_DENOMINATOR 65535 //0xffff

/**
 * @brief Structure type to set the parameters of the PID controller.
 */
typedef struct {
    int int8_P;
    int int8_I;
    int int8_D;
    int int16_P_error_limit;
    int int16_I_error_limit;
    int int16_integral_limit;
    int int16_cmd_limit;
    int int16_feedback_1n;
    int int16_error_integral;
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
void pid_init(int int8_P, int int8_I, int int8_D, int int16_P_error_limit, int int16_I_error_limit,
              int int16_itegral_limit, int int32_cmd_limit, int int16_T_s, PIDparam &param);




void pid_set_coefficients(int int8_P, int int8_I, int int8_D, PIDparam &param);

void pid_set_limits(int int16_P_error_limit, int int16_I_error_limit, int in16_itegral_limit, int int16_cmd_limit, PIDparam &param);

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
int pid_update(int int16_setpoint, int int16_feedback, int int16_T_s, PIDparam &param);
