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

/**
* Structure type for PWM ports
*
*
*/
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
 * @brief Service to generate center-aligned inverted pair PWM signals,
 *                  it provides additional ADC synchronization
 *
 * This service includes a port which triggers the ADC measurement
 *
 * @param ports Structure containing ports and other hardware information.
 * @param c_adc_trig Channel for communication with the ADC service.
 * @param c_pwm Channel for communication with the service client.
 *
 */
void pwm_triggered_service(PwmPorts &ports, chanend c_adc_trig, chanend c_pwm);

/**
 * @brief Service to generate center-aligned inverted pair PWM signals
 *
 * @param ports Structure containing ports and other hardware information.
 * @param c_pwm Channel for communication with the service client.
 */
void pwm_service(PwmPorts &ports, chanend c_pwm);
