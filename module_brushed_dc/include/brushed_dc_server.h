
/**
 * \file
 * \brief Brushed Motor Drive Server
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.9beta
 * \date 10/04/2014
 */


#include <bldc_motor_config.h>
#include <watchdog.h>

/**
 * \brief Brushed DC Drive Loop
 *
 *  Input
 * \channel c_watchdog channel for controlling the watchdog
 * \channel c_signal channel for signaling after initialization of drive loop
 * \channel c_voltage_p1 channel to receive motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * \channel c_voltage_p2 channel to receive motor voltage input value - priority 2
 * \channel c_voltage_p3 channel to receive motor voltage input value - priority 3
 *
 *  Output
 * \channel c_pwm_ctrl channel to set pwm level output to motor phases
 * \port p_ifm_esf_rstn_pwml_pwmh port to configure motor FET driver
 * \port p_ifm_coastn port to enable motor FET driver
 *
 */
void bdc_loop(chanend c_watchdog, chanend c_signal, chanend  c_voltage_p1, chanend  c_voltage_p2, \
		chanend  c_voltage_p3, chanend c_pwm_ctrl,	out port p_ifm_esf_rstn_pwml_pwmh, \
		port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2);
