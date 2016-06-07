/**
 * @file control_loops_common.h
 * @brief Common declarations for control loops
 */

#pragma once

/**
 * @brief Denominator for PID contants. The values set by the user for such constants will be divided by this value (10000 by default).
 */
#define PID_DENOMINATOR 10000.0

/**
 * @brief Structure definition for a Control Loop Service configuration.
 */
typedef struct {
    int Kp_n; /**< Value for proportional coefficient (Kp) in PID controller. Kp = Kp_n/PID_DENOMINATOR (by default PID_DENOMINATOR = 10000) */
    int Ki_n; /**< Value for integral coefficient (Ki) in PID controller. Ki = Ki_n/PID_DENOMINATOR (by default PID_DENOMINATOR = 10000) */
    int Kd_n; /**< Value for differential coefficient (Kd) in PID controller. Kd = Kd_n/PID_DENOMINATOR (by default PID_DENOMINATOR = 10000) */
    int control_loop_period; /**< Period for the control loop [microseconds]. */
    int feedback_sensor; /**< Sensor used for position control feedback [HALL_SENSOR, QEI_SENSOR]*/
    int cascade_with_torque; /**< Add torque controller at the end of velocity controller (only possible with FOC) [0, 1]*/
} ControlConfig;



/**
 * @brief Structure definition for a Control Loop Service configuration.
 */
typedef struct {
    int control_loop_period; /**< Period for the control loop [microseconds]. */

    int int21_min_position;
    int int21_max_position;
    int int10_P_position;
    int int10_I_position;
    int int10_D_position;
    int int21_P_error_limit_position;
    int int21_I_error_limit_position;
    int int22_integral_limit_position;

    int int21_max_speed;
    int int10_P_velocity;
    int int10_I_velocity;
    int int10_D_velocity;
    int int21_P_error_limit_velocity;
    int int21_I_error_limit_velocity;
    int int22_integral_limit_velocity;

    int int21_max_torque;

} PosVelocityControlConfig;
