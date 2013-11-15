
/**
 * \file torque_ctrl_server.h
 *
 *	Torque Control Loop Server
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "comm_loop_client.h"
#include <bldc_motor_config.h>
#include <internal_config.h>


/**
 * \brief Torque Control Loop
 *
 *  Input
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param sensor_used specify the sensors to used via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_adc channel to receive to torque information from current sensor
 * \channel c_hall channel to receive position information from hall
 * \channel sync_output channel to receive synced position information from qei synced with hall
 * \channel c_torque channel to receive/send torque control information
 *
 *  Output Channel
 * \channel signal_adc channel for signaling to start adc after initialization
 * \channel c_commutation channel to send motor voltage input value
 *
 */

void torque_control(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		int sensor_select, chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl);

