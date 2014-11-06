
/**
 * @file profile_control.h
 * @brief Profile Control functions
 *  Implements position profile control, velocity profile control
 *  and torque profile control functions
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#pragma once

#include <xs1.h>
#include <platform.h>

/**
 * @brief Set profile position with Position Control loop
 *
 * @Output
 * @param c_position_ctrl for communicating with the Position Control Server
 *
 * @Input
 * @param target_position is the new target position in (ticks)
 * @param velocity in (rpm)
 * @param acceleration in (rpm/s)
 * @param deceleration in (rpm/s)
 *
 */
void set_profile_position( int target_position, int velocity, int acceleration, int deceleration,
                           int sensor_select, chanend c_position_ctrl );

/**
 * @brief Set profile velocity with Velocity Control loop
 *
 * @Output
 * @param c_velocity_ctrl for communicating with the Velocity Control Server
 *
 * @Input
 * @param target_velocity is the new target velocity in (rpm)
 * @param acceleration in (rpm/s)
 * @param deceleration in (rpm/s)
 * @param max_profile_velocity is max velocity for the profile in (rpm)
 *
 */
void set_profile_velocity( int target_velocity, int acceleration, int deceleration,
                           int max_profile_velocity, chanend c_velocity_ctrl );

/**
 * @brief Set profile torque with Torque Control loop
 *
 * @Output
 * @param c_torque_ctrl for communicating with the Torque Control Server
 *
 * @Input
 * @param target_torque is the new target torque in (mNm * current resolution)
 * @param torque_slope in (mNm/s * current resolution)
 * @param cst_params struct defines cyclic synchronous torque params
 *
 */
void set_profile_torque( int target_torque, int torque_slope,
                         cst_par & cst_params, chanend c_torque_ctrl );

