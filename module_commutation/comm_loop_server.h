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

#include "dc_motor_config.h"
#include "comm_loop_client.h"

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
 * \param hall_params struct defines the pole-pairs and gear ratio
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 * \param commutation_params struct defines the commutation angle parameters
 *
 */
void commutation_sinusoidal(chanend c_hall, chanend c_qei,\
		chanend c_signal, chanend c_sync, chanend  c_commutation_p1, chanend  c_commutation_p2,\
		chanend  c_commutation_p3, chanend c_pwm_ctrl, hall_par &hall_params,\
		qei_par &qei_params, commutation_par &commutation_params);



