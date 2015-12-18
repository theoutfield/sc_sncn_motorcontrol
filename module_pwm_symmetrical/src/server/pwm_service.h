/*
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and/or its licensors.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/
#pragma once

#include <pwm_common.h>
#include <xs1.h>

typedef struct{
    buffered out port:32 p_pwm[3];
    buffered out port:32 p_pwm_inv[3];
    buffered out port:32 ?p_pwm_phase_d;
    buffered out port:32 ?p_pwm_phase_d_inv;
    clock clk;
    in port ?dummy_port;
} PwmPorts;


void disable_fets(PwmPorts &ports);

/**
 * @brief Implementation of the centre aligned inverted pair PWM server, with ADC synchronization
 *
 * This server includes a port which triggers the ADC measurement
 *
 * @param c_pwm the control channel for setting PWM values
 * @param c_adc_trig the control channel for triggering the ADC
 * @param dummy_port a dummy port used for precise timing of the ADC trigger
 * @param p_pwm the array of PWM ports
 * @param p_pwm_inv the array of inverted PWM ports
 * @param clk a clock for generating accurate PWM timing
 */
void pwm_triggered_service(PwmPorts &ports, chanend c_adc_trig, chanend c_pwm);

/**
 * @brief Implementation of the centre aligned inverted pair PWM server
 *
 * @param c_pwm the control channel for setting PWM values
 * @param p_pwm the array of PWM ports
 * @param p_pwm_inv the array of inverted PWM ports
 * @param clk a clock for generating accurate PWM timing
 */
void pwm_service( PwmPorts &ports, chanend c_pwm);
