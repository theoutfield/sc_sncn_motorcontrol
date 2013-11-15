
/**
 * \file profile_control.h
 *
 * \brief Profile Control functions
 * 	Implements position profile control, velocity profile control, and torque profile control functions
 *
 *
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and Synapticon GmbH.
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com>
 *
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code arse still covered by the
 * copyright notice above.
 *
 **/

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
 *
 */
void set_profile_position(int target_position, int velocity, int acceleration, int deceleration, chanend c_position_ctrl);

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
 * \channel c_torque_ctrl for communicating with the Velocity Control Server
 *
 *  Input
 * \param target_torque is the new target torque in (mNm * current resolution)
 * \param torque_slope in (mNm/s * current resolution)
 * \param cst_params struct defines cyclic synchronous torque params
 *
 */
void set_profile_torque(int target_torque, int torque_slope, cst_par &cst_params, chanend c_torque_ctrl);

#endif /* _PROFILE_CONTROL_H_ */
