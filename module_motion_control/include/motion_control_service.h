/**
 * @file  motion_control_service.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>

/**
 * @brief Denominator for PID contants. The values set by the user for such constants will be divided by this value (10000 by default).
 */
#define PID_DENOMINATOR 10000.0

/**
 * @brief threshold in ticks to re-enable the position controler if the limit reached.
 */
#define POSITION_LIMIT_THRESHOLD        20000

/**
 * @brief Period for the control loop [microseconds].
 */
#define POSITION_CONTROL_LOOP_PERIOD    1000


/**
 * @brief Position/Velocity control strategie
 */
typedef enum {
    POS_PID_CONTROLLER                      = 101,
    POS_PID_VELOCITY_CASCADED_CONTROLLER    = 102,
    NL_POSITION_CONTROLLER                  = 103,
    VELOCITY_PID_CONTROLLER                 = 201
} MotionControlStrategies;

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

    int position_control_strategy;      /**< Value for selecting between defferent types of position controllers or velocity controller. */
    int motion_profile_type;            /**< Value for selecting between different types of profilers (including torque/velocity/posiiton controllers. */

    int min_pos_range_limit;            /**< Value for setting the minimum position range */
    int max_pos_range_limit;            /**< Value for setting the maximum position range */
    int max_motor_speed;                /**< Value for setting the maximum motor speed */
    int max_torque;                     /**< Value for setting the maximum torque command which will be sent to torque controller */

    int enable_profiler;                /**< Value for enabling/disabling the profiler */
    int max_acceleration_profiler;      /**< Value for setting the maximum acceleration in profiler mode */
    int max_speed_profiler;             /**< Value for setting the maximum speed in profiler mode */
    int max_torque_rate_profiler;       /**< Value for setting the maximum torque in profiler mode */

    int position_kp;                    /**< Value for position controller p-constant */
    int position_ki;                    /**< Value for position controller i-constant */
    int position_kd;                    /**< Value for position controller d-constant */
    int position_integral_limit;        /**< Value for integral limit of position pid controller */

    int velocity_kp;                    /**< Value for velocity controller p-constant */
    int velocity_ki;                    /**< Value for velocity controller i-constant */
    int velocity_kd;                    /**< Value for velocity controller d-constant */
    int velocity_integral_limit;        /**< Value for integral limit of velocity pid controller */

    int k_fb;                           /**< Value for setting the feedback position sensor gain */
    int resolution;                     /**< Value for setting the resolution of position sensor [ticks/rotation] */
    int k_m;                            /**< Value for setting the gain of torque actuator */
    int moment_of_inertia;              /**< Value for setting the moment of inertia */
    int polarity;                       /**< Value for setting the polarity of the movement */
    int special_brake_release;
    int brake_shutdown_delay;

    int dc_bus_voltage;                 /**< Value for setting the nominal (rated) value of dc-link */
    int pull_brake_voltage;             /**< Value for setting the voltage for pulling the brake out! */
    int pull_brake_time;                /**< Value for setting the time of brake pulling */
    int hold_brake_voltage;             /**< Value for setting the brake voltage after it is pulled */
} MotionControlConfig;

/**
 * @brief Interface type to communicate with the Position Control Service.
 */
interface PositionVelocityCtrlInterface{

    void disable();

    void enable_position_ctrl(int pos_control_mode_);
//    void set_position(int in_target_position);
//    void set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//    void set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
//    void set_position_limits(int position_min_limit, int position_max_limit);

    void enable_velocity_ctrl(void);
//    void set_velocity(int in_target_velocity);
//    void set_offset_torque(int offset_torque_);
//    void set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//    void set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
//    void set_velocity_limits(int velocity_min_limit, int velocity_max_limit);

    /**
     * @brief (internal) Settings to suppress the overshoot
     *
     */
    void set_j(int j);

    void enable_torque_ctrl();
    void set_torque(int in_target_torque);
//    void set_torque_limits(int torque_min_limit, int torque_max_limit);


    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    MotionControlConfig get_position_velocity_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_position_velocity_control_config(MotionControlConfig in_config);

    /**
     * @brief Setter for new configuration in the Motorcontrol Service.
     *
     * @param in_config New Service configuration.
     */
    void set_motorcontrol_config(MotorcontrolConfig in_config);

    /**
     * @brief Getter for current configuration used by the Motorcontrol Service.
     *
     * @return Current Service configuration.
     */
    MotorcontrolConfig get_motorcontrol_config();

    /**
     * @brief Sets brake status to ON (no movement) or OFF (possible to move)
     */
    void set_brake_status(int brake_status);


    void update_brake_configuration();

    /**
     * @brief Enables the offset detection process
     */
    MotorcontrolConfig set_offset_detection_enabled();

    /**
     * @brief Send a reset fault command to the motorcontrol
     */
    void reset_motorcontrol_faults();

    int get_position();

    int get_velocity();

    UpstreamControlData update_control_data(DownstreamControlData downstream_control_data_);
};


/**
 * @brief Initializer helper for the Position Control Service.
 *        It is required the client to call this function before
 *        starting to perform position control.
 *
 * @param i_position_control Communication interface to the Position Control Service.
 */
void init_position_velocity_control(interface PositionVelocityCtrlInterface client i_position_control);

/**
 * @brief Service to perform a Position PID Control Loop on top of a Motor Control Service.
 *        You will need a Motor Control Stack running parallel to this Service,
 *        have a look at Motor Control Service for more information.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param position_ctrl_config Configuration for the Position Control Service.
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param i_position_control Array of communication interfaces to handle up to 3 different clients.
 */
void motion_control_service(int app_tile_usec, MotionControlConfig &pos_velocity_control_config,
                    interface MotorControlInterface client i_motorcontrol,
                    interface PositionVelocityCtrlInterface server i_position_control[3],
                    client interface UpdateBrake i_update_brake);
