
/**
 * \file  torque_ctrl_client.h
 * \brief Torque Control Loop Client functions
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

/**
 * \brief Initialise Torque Control Loop
 *
 *  Input Channel
 * \channel c_torque_ctrl channel to signal initialisation
 */
int init_torque_control(chanend c_torque_ctrl);

/**
 * \brief Checks Torque Control Loop Status
 *
 *  Input Channel
 * \channel c_torque_ctrl channel to get state of control loop
 *
 *  Output
 * \return state of the control loop : 1 - active, 0 - inactive
 */
int check_torque_ctrl_state(chanend c_torque_ctrl);

/**
 * \brief Torque Limiter
 *
 *  Input
 * \param torque is the input torque to be limited in range
 * \param max_torque_limit is the max torque that can be reached
 *
 *  Output
 * \return torque in the range [-max_torque_limit to max_torque_limit] (mNm * Current resolution)
 */
int torque_limit(int torque, int max_torque_limit);

/**
 * \brief Get actual torque from Torque Control Loop
 *
 *  Output Channel
 * \channel c_torque channel to receive actual torque
 *
 *  Output
 * \return actual torque from torque control in range [0 - mNm * Current Resolution]
 */
int get_torque(chanend c_torque_ctrl);

/**
 * \brief Set new target torque for Torque Control Loop
 *
 *  Input Channel
 * \channel c_torque channel to signal new target torque
 *
 *  Input
 * \param torque is the new target torque range [0 - mNm * Current Resolution]
 */
void set_torque(int torque, chanend c_torque_ctrl);

/**
 * \brief Set Torque Control PID Parameters
 *
 *  Input
 * \param torque_ctrl_params struct defines the torque control PID parameters
 */
void set_torque_ctrl_param(ctrl_par &torque_ctrl_params, chanend c_torque_ctrl);

/**
 * \brief Set hall sensor parameters for Torque Control
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void set_torque_ctrl_hall_param(hall_par &hall_params, chanend c_torque_ctrl);

/**
 * \brief Set QEI sensor parameters for Torque Control
 *
 * \param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_torque_ctrl_qei_param(qei_par &qei_params, chanend c_torque_ctrl);

/**
 * \brief Sets the sensor used for Torque Control
 *
 * \param sensor_used defines the sensor to be used (HALL/QEI) for Torque Control
 */
void set_torque_sensor(int sensor_used, chanend c_torque_ctrl);

/**
 * \brief Enables Torque Control mode operation
 *
 * \channel c_torque_ctrl channel to signal enable torque control
 */
void enable_torque_ctrl(chanend c_torque_ctrl);

/**
 * \brief Shutdown Torque Control mode operation
 *
 * \channel c_torque_ctrl channel to signal shutdown of torque control
 */
void shutdown_torque_ctrl(chanend c_torque_ctrl);


/**
 * \brief Set new target torque for torque control (advanced function)
 *
 *  Input Channel
 * \channel c_torque_ctrl channel to signal new target torque
 *
 *  Input
 * \param cst_param struct defines the motor parameters and torque limits
 * \param target_torque is the new target torque
 * \param torque_offset defines offset in torque
 */
void set_torque_cst(cst_par &cst_params, int target_torque, int torque_offset, chanend c_torque_ctrl);

