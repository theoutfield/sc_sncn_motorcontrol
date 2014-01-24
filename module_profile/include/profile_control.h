
/**
 *
 * \file profile_control.h
 *
 * \brief Profile Control functions
 * 	Implements position profile control, velocity profile control
 * 	and torque profile control functions
 *
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
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

#ifndef _PROFILE_CONTROL_H_
#define _PROFILE_CONTROL_H_


#include <xs1.h>
#include <platform.h>
#include <bldc_motor_config.h>

/**
 * \brief Set profile position with Position Control loop
 *
 *  Output channel
 * \channel c_position_ctrl for communicating with the Position Control Server
 *
 *  Input
 * \param target_position is the new target position in (degree)
 * \param velocity in (rpm)
 * \param acceleration in (rpm/s)
 * \param deceleration in (rpm/s)
 * \param min position in (degree)
 * \param max position in (degree)
 *
 */
void set_profile_position(int target_position, int velocity, int acceleration, int deceleration,\
		int max_position, int min_position, chanend c_position_ctrl);

/**
 * \brief Set profile velocity with Velocity Control loop
 *
 *  Output channel
 * \channel c_velocity_ctrl for communicating with the Velocity Control Server
 *
 *  Input
 * \param target_velocity is the new target velocity in (rpm)
 * \param acceleration in (rpm/s)
 * \param deceleration in (rpm/s)
 * \param max_profile_velocity is max velocity for the profile in (rpm)
 *
 */
void set_profile_velocity(int target_velocity, int acceleration, int deceleration, int max_profile_velocity, chanend c_velocity_ctrl);

/**
 * \brief Set profile torque with Torque Control loop
 *
 *  Output channel
 * \channel c_torque_ctrl for communicating with the Torque Control Server
 *
 *  Input
 * \param target_torque is the new target torque in (mNm * current resolution)
 * \param torque_slope in (mNm/s * current resolution)
 * \param cst_params struct defines cyclic synchronous torque params
 *
 */
void set_profile_torque(int target_torque, int torque_slope, cst_par &cst_params, chanend c_torque_ctrl);

#endif /* _PROFILE_CONTROL_H_ */
