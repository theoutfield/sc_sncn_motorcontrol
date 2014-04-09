
/**
 * \file  torque_ctrl_server.h
 * \brief Torque Control Loop Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "comm_loop_client.h"
#include <bldc_motor_config.h>
#include <internal_config.h>


/**
 * \brief Torque Control Loop
 *
 *  Input
 * \param torque_ctrl_params struct defines the torque control parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_select specify the sensor to use via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_adc channel to receive to torque information from current sensor
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_torque channel to receive/send torque control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */

void torque_control(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		int sensor_used, chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl);

