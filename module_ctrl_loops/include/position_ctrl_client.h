
/**
 * \file  position_ctrl_client.h
 * \brief Position control Loop Client functions
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

#include <bldc_motor_config.h>
#include <internal_config.h>
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"


/**
 * \brief Initialise Position Control Loop
 *
 *  Input Channel
 * \channel c_position_ctrl channel to signal initialisation
 */
int init_position_control(chanend c_position_ctrl);

/**
 * \brief Checks Position Control Loop Status
 *
 *  Input Channel
 * \channel c_position_ctrl channel to get state of control loop
 *
 *  Output
 * \return state of the control loop : 1 - active, 0 - inactive
 */
int check_position_ctrl_state(chanend c_position_ctrl);

/**
 * \brief Position Limiter
 *
 *  Input
 * \param position is the input position to be limited in range
 * \param max_position_limit is the max position that can be reached
 * \param min_position_limit is the min position that can be reached
 *
 *  Output
 * \return position in the range [min_position_limit - max_position_limit]
 */
int position_limit(int position, int max_position_limit, int min_position_limit);


/**
 * \brief Set new target position for position control
 *
 *  Input Channel
 * \channel c_position_ctrl channel to signal new target position
 *
 *  Input
 * \param target_position is the new target position
 */
void set_position(int target_position, chanend c_position_ctrl);

/**
 * \brief Get actual position from position control
 *
 *  Output Channel
 * \channel c_position_ctrl channel to receive actual position
 *
 *  Output
 * \return actual position from position control
 */
int get_position(chanend c_position_ctrl);

/**
 * \brief Set Position Control PID Parameters
 *
 *  Input
 * \param position_ctrl_params struct defines the position control PID parameters
 */
void set_position_ctrl_param(ctrl_par &position_ctrl_params, chanend c_position_ctrl);

/**
 * \brief Set hall sensor parameters for Position Control
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void set_position_ctrl_hall_param(hall_par &hall_params, chanend c_position_ctrl);

/**
 * \brief Set QEI sensor for Position Control
 *
 * \param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_position_ctrl_qei_param(qei_par &qei_params, chanend c_position_ctrl);

/**
 * \brief Sets the sensor used for Position Control
 *
 * \param sensor_used defines the sensor to be used (HALL/QEI) for Position Control
 */
void set_position_sensor(int sensor_used, chanend c_position_ctrl);

/**
 * \brief Enables Position Control mode operation
 *
 * \channel c_position_ctrl channel to signal enable position control
 */
void enable_position_ctrl(chanend c_position_ctrl);

/**
 * \brief Shutdown Position Control mode operation
 *
 * \channel c_position_ctrl channel to signal shutdown of position control
 */
void shutdown_position_ctrl(chanend c_position_ctrl);

/**
 * \brief Set new target position for position control (advanced function)
 *
 *  Input Channel
 * \channel c_position_ctrl channel to signal new target position
 *
 *  Input
 * \param csp_param struct defines the motor parameters and position limits
 * \param target_position is the new target position
 * \param position_offset defines offset in position
 * \param velocity_offset defines offset in velocity
 * \param torque_offset defines offset in torque
 */
void set_position_csp(csp_par &csp_params, int target_position, int position_offset, int velocity_offset,\
		              int torque_offset, chanend c_position_ctrl);

