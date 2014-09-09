/**
 * @file
 * @brief Brushed Motor Drive Server
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

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
void bdc_loop(chanend c_watchdog, chanend c_signal,
              chanend ? c_voltage_p1, chanend ? c_voltage_p2, chanend ? c_voltage_p3,
              chanend c_pwm_ctrl,
              out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn,
              port p_ifm_ff1, port p_ifm_ff2);
