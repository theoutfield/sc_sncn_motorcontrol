/**
 * @file commutation_server.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <commutation_client.h>
#include <watchdog.h>
#include <hall_client.h>
#include <qei_client.h>
#include <rotary_sensor.h>
#include <bldc_motor_config.h>

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
 * @param[in] hall_params struct defines the pole-pairs and gear ratio
 * @param[in] qei_params the struct defines sensor type and resolution parameters for QEI
 * @param[in] commutation_params struct defines the commutation angle parameters
 *
 */
[[combinable]]
void commutation_sinusoidal(chanend ?c_hall, chanend ?c_qei, client interface AMS ?i_ams, chanend ?c_signal, chanend ? c_watchdog,
                            chanend ? c_commutation_p1, chanend ? c_commutation_p2, chanend ? c_commutation_p3, chanend c_pwm_ctrl,
                            out port ? p_ifm_esf_rstn_pwml_pwmh, port ? p_ifm_coastn, port ? p_ifm_ff1, port ? p_ifm_ff2,
                            hall_par &hall_params, qei_par & qei_params, commutation_par & commutation_params);

/**
 * @brief Initialize commutation parameters
 *
 * @param commutation_params struct defines the commutation angle parameters
 * @param hall_params struct defines the pole-pairs and gear ratio
 * @param nominal_speed is the rated speed for the motor given on specs sheet
 */
void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);


