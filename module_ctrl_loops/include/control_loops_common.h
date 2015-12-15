/**
 * @file control_loops_common.h
 * @brief Common declarations for control loops
 */

#pragma once

/**
 * @brief Struct definition for PID Controller
 */
typedef struct {
    int Kp_n, Kp_d; /**< Kp = Kp_n/Kp_d */
    int Ki_n, Ki_d; /**< Ki = Ki_n/Ki_d */
    int Kd_n, Kd_d; /**< Kd = Kd_n/Kd_d */
    int Integral_limit; /**< Lorem ipsum... */
    int Control_limit; /**< Lorem ipsum... */
    int Loop_time; /**< Lorem ipsum... */
    int sensor_used; /**< Lorem ipsum... */
} ControlConfig;

/**
 * @brief Struct definition for Synchronous torque param
 */
typedef struct
{
    int nominal_motor_speed; /**< Lorem ipsum... */
    int nominal_current; /**< Lorem ipsum... */
    int motor_torque_constant; /**< Lorem ipsum... */
    int max_torque; /**< Lorem ipsum... */
    int polarity; /**< Lorem ipsum... */
} CyclicSyncTorqueConfig;

/**
 * @brief Struct definition for Synchronous velocity param
 */
typedef struct
{
    int max_motor_speed; /**< Lorem ipsum... */
    int nominal_current; /**< Lorem ipsum... */
    int motor_torque_constant; /**< Lorem ipsum... */
    int polarity; /**< Lorem ipsum... */
    int max_acceleration; /**< Lorem ipsum... */
} CyclicSyncVelocityConfig;

/**
 * @brief Struct definition for Synchronous position param
 */
typedef struct
{
    CyclicSyncVelocityConfig velocity_config; /**< Lorem ipsum... */
    int max_following_error; /**< Lorem ipsum... */
    int max_position_limit; /**< Lorem ipsum... */
    int min_position_limit; /**< Lorem ipsum... */
} CyclicSyncPositionConfig;
