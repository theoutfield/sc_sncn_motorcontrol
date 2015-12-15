/**
 * @file
 * @brief Brushed Motor Drive Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>

/**
 * @brief Brushed DC Drive Loop
 *
 * @Input
 * @param c_watchdog channel for controlling the watchdog
 * @param c_signal channel for signaling after initialization of drive loop
 * @param c_voltage_p1 channel to receive motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * @param c_voltage_p2 channel to receive motor voltage input value - priority 2
 * @param c_voltage_p3 channel to receive motor voltage input value - priority 3
 *
 * @Output
 * @param c_pwm_ctrl channel to set pwm level output to motor phases
 * @param p_ifm_esf_rstn_pwml_pwmh port to configure motor FET driver
 * @param p_ifm_coastn port to enable motor FET driver
 * @param p_ifm_ff1 FET driver fault flag 1
 * @param p_ifm_ff2 FET driver fault flag 2
 */
void bdc_loop(chanend c_pwm_ctrl,
               interface WatchdogInterface client watchdog_interface,
               interface MotorcontrolInterface server commutation_interface[5],
               FetDriverPorts &fet_driver_ports,
               MotorcontrolConfig &commutation_params);
