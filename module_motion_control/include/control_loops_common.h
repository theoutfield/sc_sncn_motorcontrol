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
#define NL_POSITION_CONTROLLER                  103
#define VELOCITY_PID_CONTROLLER                 201



/**
 * @brief Structure definition for a Control Loop Service configuration.
 */
typedef struct {

    int position_control_strategy;
    int motion_profile_type;

    int min_pos_range_limit;
    int max_pos_range_limit;
    int max_motor_speed;
    int max_torque;

    int enable_profiler;
    int max_acceleration_profiler;
    int max_speed_profiler;
    int max_torque_rate_profiler;

    int position_kp;
    int position_ki;
    int position_kd;
    int position_integral_limit;

    int velocity_kp;
    int velocity_ki;
    int velocity_kd;
    int velocity_integral_limit;

    int position_fc;
    int velocity_fc;

    int k_fb; //position feedback gain [milli-ticks/rad]
    int resolution;
    int k_m;  //gain of torque actuator
    int moment_of_inertia;
    int polarity; //polarity of the movement [-1/1]
    int special_brake_release;
    int brake_shutdown_delay;

    int dc_bus_voltage;       //in volts
    int pull_brake_voltage; //in milli volts
    int pull_brake_time;    //in milli seconds
    int hold_brake_voltage; //in milli volts

} MotionControlConfig;




