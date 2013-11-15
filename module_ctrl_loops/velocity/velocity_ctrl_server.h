
/**
 * \file velocity_ctrl_server.h
 *
 *	Velocity Control Loop Server
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

#include <bldc_motor_config.h>
#include <internal_config.h>


/**
 * \brief Velocity Control Loop
 *
 *  Input
 * \param velocity_ctrl_params struct defines the velocity control parameters
 * \param sensor_filter_par struct defines the filter parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_used specify the sensors to used via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_velocity_ctrl channel to receive/send velocity control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void velocity_control(ctrl_par &velocity_ctrl_params, filter_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params, \
	 	 	 int sensor_used, chanend c_hall, chanend c_qei, chanend c_velocity_ctrl, chanend c_commutation);

