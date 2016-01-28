/**
 * @file commutation_server.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>

/**
 * @brief Sinusoidal based Commutation Loop
 *
 * @param[in] c_hall A chanend connected to the hall server
 * @param[in] c_qei A chanend connected to the qei server (QEI)
 * @param[in] c_signal A chanend for signaling after initialization of commutation loop
 * @param[in] c_watchdog A chanend connected to the watchdog
 * @param[in] c_commutation_p1 channel for receiving motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * @param[in] c_commutation_p2 channel for receiving motor voltage input value - priority 2
 * @param[in] c_commutation_p3 channel for receiving motor voltage input value - priority 3
 * @param[out] c_pwm_ctrl channel to set PWM level output to motor phases
 * @param[out] p_ifm_esf_rstn_pwml_pwmh port to configure motor FET driver
 * @param[out] p_ifm_coastn port to enable motor FET driver
 * @param[out] p_ifm_ff1
 * @param[out] p_ifm_ff2
 * @param[in] hall_config struct defines the pole-pairs and gear ratio
 * @param[in] qei_params the struct defines sensor type and resolution parameters for QEI
 * @param[in] commutation_params struct defines the commutation angle parameters
 * @param[in] sensor_select used for commutation
 *
 */
[[combinable]]
void bldc_loop(HallConfig hall_config, QEIConfig qei_config,
                            interface HallInterface client i_hall, interface QEIInterface client ?i_qei,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[4],
                            chanend c_pwm_ctrl,
                            FetDriverPorts &fet_driver_ports,
                            MotorcontrolConfig &commutation_params);
