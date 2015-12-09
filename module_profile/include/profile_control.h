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
 * @brief struct definition for profile velocity param
 */
typedef struct
{
    int max_profile_velocity;
    int max_acceleration;
    int profile_acceleration;
    int profile_deceleration;
    int quick_stop_deceleration;
    int polarity;
} ProfileVelocityConfig;

/**
 * @brief struct definition for profile position param
 */
typedef struct
{
    ProfileVelocityConfig velocity_config;
    int profile_velocity;
    int software_position_limit_min;
    int software_position_limit_max;
    int max_acceleration;
} ProfilePositionConfig;

/**
 * @brief struct definition for profile torque param
 */
typedef struct
{
    int profile_slope;
    int max_torque;
    int polarity;
} ProfileTorqueConfig;

void init_position_profiler(ProfilePositionConfig profile_position_config,
                                interface PositionControlInterface client i_position_control);

void init_velocity_profiler(ProfileVelocityConfig profile_velocity_config,
                                interface VelocityControlInterface client i_velocity_control);

void init_torque_profiler(ProfileTorqueConfig profile_torque_config,
                                interface TorqueControlInterface client i_torque_control);

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
                           interface PositionControlInterface client i_position_control );

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
                           interface VelocityControlInterface client i_velocity_control );

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
                         interface TorqueControlInterface client i_torque_control );

