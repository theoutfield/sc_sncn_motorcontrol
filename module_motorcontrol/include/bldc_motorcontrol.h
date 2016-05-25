/**
 * @file bldc_motorcontrol.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>
#include <adc_service.h>

/**
 * @brief Sinusoidal and FOC based Commutation Loop
 *
 * @param hall_config Structure defines the pole-pairs and gear ratio
 * @param qei_config the Structure defines sensor type and resolution parameters for QEI
 * @param i_hall Interface to hall Service
 * @param i_qei Interface to Incremental Encoder Service (QEI)
 * @param i_biss Interface to BiSS Encoder Service (QEI)
 * @param i_watchdog Interface to watchdog
 * @param i_motorcontrol Array of interfaces towards clients
 * @param c_pwm_ctrl channel to set PWM level output to motor phases
 * @param fet_driver_ports Structure containing FED driver ports
 * @param commutation_params Structure defines the commutation angle parameters
 *
 */
[[combinable]]
void bldc_loop(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            interface MotorcontrolInterface server i_motorcontrol[4],
        chanend c_pwm_ctrl, interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface BrakeInterface client ?i_brake);

void space_vector_pwm(int umot, int angle,  int pwm_on_off, unsigned pwmout[]);
