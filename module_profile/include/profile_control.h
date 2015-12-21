/**
 * @file profile_control.h
 * @brief Profile Control functions
 *  Implements position profile control, velocity profile control
 *  and torque profile control functions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <position_ctrl_service.h>
#include <velocity_ctrl_service.h>
#include <torque_ctrl_service.h>

/**
 * @brief Lorem ipsum...
 *
 * @param profile_position_config Lorem ipsum...
 * @param i_position_control Lorem ipsum...
 */
void init_position_profiler(ProfilerConfig profile_position_config,
                                interface PositionControlInterface client i_position_control);

/**
 * @brief Lorem ipsum...
 *
 * @param profile_velocity_config Lorem ipsum...
 * @param i_velocity_control Lorem ipsum...
 */
void init_velocity_profiler(ProfilerConfig profile_velocity_config,
                                interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Lorem ipsum...
 *
 * @param profile_torque_config Lorem ipsum...
 * @param i_torque_control Lorem ipsum...
 */
void init_torque_profiler(ProfilerConfig profile_torque_config,
                                interface TorqueControlInterface client i_torque_control);

/**
 * @brief Set profile position with Position Control loop
 *
 * @param target_position is the new target position in (ticks)
 * @param velocity in (rpm)
 * @param acceleration in (rpm/s)
 * @param deceleration in (rpm/s)
 * @param i_position_control for communicating with the Position Control Server
 *
 */
void set_profile_position( int target_position, int velocity, int acceleration, int deceleration,
                           interface PositionControlInterface client i_position_control );

/**
 * @brief Set profile velocity with Velocity Control loop
 *
 * @param target_velocity is the new target velocity in (rpm)
 * @param acceleration in (rpm/s)
 * @param deceleration in (rpm/s)
 * @param i_velocity_control for communicating with the Velocity Control Server
 *
 */
void set_profile_velocity( int target_velocity, int acceleration, int deceleration,
                           interface VelocityControlInterface client i_velocity_control );

/**
 * @brief Set profile torque with Torque Control loop
 *
 * @param target_torque is the new target torque in (mNm * current resolution)
 * @param torque_slope in (mNm/s * current resolution)
 * @param i_torque_control for communicating with the Torque Control Server
 *
 */
void set_profile_torque( int target_torque, int torque_slope,
                         interface TorqueControlInterface client i_torque_control );

