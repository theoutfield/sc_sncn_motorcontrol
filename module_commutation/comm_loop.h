
/**
 * \file comm_loop.h
 *
 *	Commutation Loop based on Space Vector PWM method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			& Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "predriver/a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include "dc_motor_config.h"

/**
* \brief Struct for commutation parameters
*/
typedef struct S_COMMUTATION {
	int angle_variance;
	int max_speed_reached;
	int qei_forward_offset;
	int qei_backward_offset;
	int offset_forward;
	int offset_backward;
	int flag;
} commutation_par;

/**
* \brief initialize commutation parameters
*
* \param commutation_params struct defines the commutation angle parameters
* \param hall_params struct defines the pole-pairs and gear ratio
* \param nominal speed is the rated speed for the motor given on specs sheet
*/
void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);

/**
* \brief initialize commutation loop
*/
int init_commutation(chanend c_signal);


/**
 * \brief Specify sensor for commutation
 *
 * \channel c_commutation channel to send information to the commutation loop
 *
 *  Input
 * \param sensor_select specify sensor used for commutation through defines HALL and QEI
 */
void commutation_sensor_select(chanend c_commutation, int sensor_select);

/**
 * \brief Sinusoidal based Commutation Loop
 *
 *  Input Channels
 * \channel c_hall channel to receive position information from hall sensor
 * \channel c_qei channel to receive position information from qei sensor
 * \channel c_signal_adc channel for signaling to start adc after initialization
 * \channel c_signal channel for signaling after initialization of commutation loop
 * \channel c_sync channel for receive synced position information *
 * \channel c_commutation_p1 channel to receive motor voltage input value - priority 1 (highest) 1 ... (lowest) 3
 * \channel c_commutation_p2 channel to receive motor voltage input value - priority 2
 * \channel c_commutation_p3 channel to receive motor voltage input value - priority 3
 *
 *  Output Channels
 * \channel c_pwm_ctrl channel to set pwm level output
 *
 *  Input
 * \param sensor_select specify sensor used for commutation through defines HALL and QEI
 * \param hall_params struct defines the pole-pairs and gear ratio
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 * \param commutation_params struct defines the commutation angle parameters
 *
 */
void commutation_sinusoidal(chanend c_hall, chanend c_qei,\
		chanend c_signal, chanend c_sync, chanend  c_commutation_p1, chanend  c_commutation_p2,\
		chanend  c_commutation_p3, chanend c_pwm_ctrl, hall_par &hall_params,\
		qei_par &qei_params, commutation_par &commutation_params);

/**
 *  \brief Set Input voltage for commutation loop
 *
 *   Output Channels
 * 	\channel c_commutation channel to send out motor voltage input value
 *
 * 	 Input
 * 	\param input_voltage is motor voltage input value to be set (range allowed -13739 to 13739)
 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage);

/**
 *  \brief Internal function used to setup the commutation parameters for correct operation
 *
 *   Output Channels
 * 	\channel c_commutation channel to send out motor voltage input value
 *
 * 	 Input
 * 	\param commutation_params struct defines the commutation angle parameters
 */
void set_commutation_params(chanend c_commutation, commutation_par &commutation_params);
