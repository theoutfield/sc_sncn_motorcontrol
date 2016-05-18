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

#include <xs1.h>

#include <pwm_common.h>
#include <pwm_ports.h>


interface BrakeInterface {
    void set_brake(int enable);
    int get_brake();
};

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
 * @param i_brake interface to the brake service.
 *
 */
void pwm_triggered_service(PwmPorts &ports, chanend c_adc_trig, chanend c_pwm, interface BrakeInterface server ?i_brake);

/**
 * @brief Service to generate center-aligned inverted pair PWM signals
 *
 * @param ports Structure containing ports and other hardware information.
 * @param c_pwm Channel for communication with the service client.
 * @param i_brake interface to the brake service.
 */
void pwm_service(PwmPorts &ports, chanend ?c_pwm, interface BrakeInterface server ?i_brake);

void brake_service(buffered out port:32 p_pwm, buffered out port:32 p_pwm_inv, interface BrakeInterface server i_brake);


