/**
 * \file position_ctrl_server.h
 *
 *	Position Control Loop Server
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
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop_client.h"

/**
 * \brief Position Control Loop
 *
 *  Input
 * \param position_ctrl_params struct defines the position control parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_used specify the sensors to used via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_position_ctrl channel to receive/send position control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_position_ctrl, chanend c_commutation);
