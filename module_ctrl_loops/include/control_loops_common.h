/**
 * @file control_loops_common.h
 * @brief Common declarations for control loops
 */

#pragma once

/**
 * @brief struct definition for PID Controller
 */
typedef struct {
    int Kp_n, Kp_d; //Kp = Kp_n/Kp_d
    int Ki_n, Ki_d; //Ki = Ki_n/Ki_d
    int Kd_n, Kd_d; //Kd = Kd_n/Kd_d
    int Integral_limit;
    int Control_limit;
    int Loop_time;
    int sensor_used;
} ControlConfig;

/**
 * @brief struct definition for Synchronous torque param
 */
typedef struct
{
    int nominal_motor_speed;
    int nominal_current;
    int motor_torque_constant;
    int max_torque;
    int polarity;
} CyclicSyncTorqueConfig;

/**
 * @brief struct definition for Synchronous velocity param
 */
typedef struct
{
    int max_motor_speed;
    int nominal_current;
    int motor_torque_constant;
    int polarity;
    int max_acceleration;
} CyclicSyncVelocityConfig;

/**
 * @brief struct definition for Synchronous position param
 */

typedef struct
{
    CyclicSyncVelocityConfig velocity_config;
    int max_following_error;
    int max_position_limit;
    int min_position_limit;
} CyclicSyncPositionConfig;
