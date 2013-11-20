
/**
 *
 * \file profile.h
 *
 * \brief Profile Generation for Position, Velocity and Torque
 * 	Implements position profile based on Linear Function with
 * 	Parabolic Blends, velocity profile and torque profiles are
 * 	based on linear functions.
 *
 *
 * Copyright (c) 2013, Synapticon GmbH
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
 * either expressed or implied, of the Synapticon GmbH Project.
 *
 */

#include <stdio.h>
#include <math.h>
#ifndef _PROFILE_H_
#define _PROFILE_H_

/*Profile Velocity Quick Stop*/

/**
 * \brief Initialise Quick Stop Velocity Profile
 *
 *  Input
 * \param actual_velocity
 * \param quick_stop_deceleration defines the deceleration for quick stop profile
 *
 * Output
 * \return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);

/**
 * \brief Generate Quick Stop Velocity Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target velocity at the step input
 */
extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/

/**
 * \brief Initialise Velocity Profile
 *
 *  Input
 * \param target_velocity
 * \param actual_velocity
 * \param acceleration for the velocity profile
 * \param deceleration for the velocity profile
 * \param max_velocity for the velocity profile
 *
 * Output
 * \return no. of steps for velocity profile : range [1 - steps]
 */
extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration, int max_velocity);

/**
 * \brief Generate Velocity Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target velocity at the step input
 */
extern int velocity_profile_generate(int step);

/*Profile Position Mode*/

/**
 * \brief Initialise Position Profile Limits
 *
 *  Input
 * \param gear_ratio
 * \param max_acceleration for the position profile
 * \param max_velocity for the position profile
 *
 */
extern void init_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity);

/**
 * \brief Initialise Position Profile
 *
 *  Input
 * \param target_position
 * \param actual_position
 * \param velocity for the position profile
 * \param acceleration for the position profile
 * \param deceleration for the position profile
 *
 * Output
 * \return no. of steps for position profile : range [1 - steps]
 */
extern int init_position_profile(int target_position, int actual_position,	int velocity, int acceleration, \
        						 int deceleration);

/**
 * \brief Generate Position Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target position at the step input
 */
extern int position_profile_generate(int step);

/*Profile Position Quick Stop*/

/**
 * \brief Initialise Quick Stop Position Profile
 *
 *  Input
 * \param actual_velocity
 * \param actual_position
 * \param max_acceleration defines the deceleration for quick stop profile
 *
 * Output
 * \return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_acceleration);

/**
 * \brief Generate Quick Stop Position Profile
 *
 *  Input
 * \param step current step of the profile
 * \param actual_velocity
 *
 * Output
 * \return corresponding target position at the step input
 */
extern int quick_stop_position_profile_generate(int steps, int actual_velocity);

/**
 * \brief Initialise Linear Profile
 *
 *  Input
 * \param target_value
 * \param actual_value
 * \param acceleration for the Linear profile
 * \param deceleration for the Linear profile
 * \param max_value for the Linear profile
 *
 * Output
 * \return no. of steps for linear profile : range [1 - steps]
 */
extern int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration, int max_value);

/**
 * \brief Generate Linear Profile
 *
 *  Input
 * \param step current step of the profile
 *
 * Output
 * \return corresponding target value at the step input
 */
extern int linear_profile_generate(int step);

#ifndef __XC__
int init_linear_profile_float(float target_value, float actual_value, float acceleration, float deceleration, float max_value);

float linear_profile_generate_float(int step);
#endif

#endif /* _PROFILE_H_ */
