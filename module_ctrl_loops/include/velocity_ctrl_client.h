
/**
 * \file velocity_ctrl_client.h
 * \brief Velocity Control Loop Client functions
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
 * \brief Initialise Velocity Control Loop
 *
 *  Input Channel
 * \channel c_velocity_ctrl channel to signal initialisation
 */
int init_velocity_control(chanend c_velocity_ctrl);

/**
 * \brief Checks Velocity Control Loop Status
 *
 *  Input Channel
 * \channel c_velocity_ctrl channel to get state of control loop
 *
 *  Output
 * \return state of the control loop : 1 - active, 0 - inactive
 */
int check_velocity_ctrl_state(chanend c_velocity_ctrl);

/**
 * \brief Velocity Limiter
 *
 *  Input
 * \param velocity is the input velocity to be limited in range
 * \param max_speed is the max speed that can be reached
 *
 *  Output
 * \return velocity in the range [-max_speed to max_speed] (rpm)
 */
int max_speed_limit(int velocity, int max_speed);

/**
 * \brief Set new target velocity for Velocity Control Loop
 *
 *  Input Channel
 * \channel c_velocity_ctrl channel to signal new target velocity
 *
 *  Input
 * \param target_velocity is the new target velocity (rpm)
 */
void set_velocity(int target_velocity, chanend c_velocity_ctrl);


/**
 * \brief Get actual velocity from Velocity Control Loop
 *
 *  Output Channel
 * \channel c_velocity_ctrl channel to receive actual velocity
 *
 *  Output
 * \return actual velocity from velocity control (rpm)
 */
int get_velocity(chanend c_velocity_ctrl);

/**
 * \brief Set Velocity Control PID Parameters
 *
 *  Input
 * \param velocity_ctrl_params struct defines the velocity control PID parameters
 */
void set_velocity_ctrl_param(ctrl_par &velocity_ctrl_params, chanend c_velocity_ctrl);

/**
 * \brief Set hall sensor parameters for Velocity Control
 *
 *	Input
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void set_velocity_ctrl_hall_param(hall_par &hall_params, chanend c_velocity_ctrl);

/**
 * \brief Set QEI sensor for Velocity Control
 *
 *	Input
 * \param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_velocity_ctrl_qei_param(qei_par &qei_params, chanend c_velocity_ctrl);

/**
 * \brief Sets the sensor used for Velocity Control
 *
 *  Input
 * \param sensor_used defines the sensor to be used (HALL/QEI) for Velocity Control
 */
void set_velocity_sensor(int sensor_used, chanend c_velocity_ctrl);

/**
 * \brief Enables Velocity Control mode operation
 *
 * \channel c_velocity_ctrl channel to signal enable velocity control
 */
void enable_velocity_ctrl(chanend c_velocity_ctrl);

/**
 * \brief Shutdown Velocity Control mode operation
 *
 * \channel c_velocity_ctrl channel to signal shutdown of velocity control
 */
void shutdown_velocity_ctrl(chanend c_velocity_ctrl);


/**
 * \brief Set new target velocity for velocity control (advanced function)
 *
 *  Input Channel
 * \channel c_velocity_ctrl channel to signal new target velocity input
 *
 *  Input
 * \param csv_param struct defines the motor parameters and velocity limits
 * \param target_velocity is the new target velocity
 * \param velocity_offset defines offset in velocity
 * \param torque_offset defines offset in torque
 */
void set_velocity_csv(csv_par &csv_params, int target_velocity,
		int velocity_offset, int torque_offset, chanend c_velocity_ctrl);



