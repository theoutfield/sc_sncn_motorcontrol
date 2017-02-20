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


#define POS_PID_CONTROLLER                      101
#define POS_PID_VELOCITY_CASCADED_CONTROLLER    102
#define NL_POSITION_CONTROLLER         103
#define VELOCITY_PID_CONTROLLER                 201



/**
 * @brief Structure definition for a Control Loop Service configuration.
 */
typedef struct {

    int control_mode;

    int min_pos;
    int max_pos;
    int max_speed;
    int max_torque;

    int enable_profiler;
    int max_acceleration_profiler;
    int max_speed_profiler;

    int P_pos;
    int I_pos;
    int D_pos;
    int integral_limit_pos;
    int pid_gain;

    int P_velocity;
    int I_velocity;
    int D_velocity;
    int integral_limit_velocity;

    int position_fc;
    int velocity_fc;

    int k_fb; //position feedback gain [milli-ticks/rad]
    int resolution;
    int k_m;  //gain of torque actuator
    int j;
    int polarity; //polarity of the movement [-1/1]
    int special_brake_release;
    int brake_shutdown_delay;

    int voltage_pull_brake;
    int time_pull_brake;
    int voltage_hold_brake;

} PosVelocityControlConfig;




