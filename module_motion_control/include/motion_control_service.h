/**
 * @file  motion_control_service.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <i2c.h>

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
#define POSITION_CONTROL_LOOP_PERIOD    333

/**
 * @brief Period for the control loop [microseconds].
 */
#define POSITION_CONTROL_LOOP_PERIOD_SEC  POSITION_CONTROL_LOOP_PERIOD * 1e-6

/**
 * @brief Threshold to detect brake release in ticks.
 */
#define BRAKE_RELEASE_THRESHOLD         4000

/**
 * @brief Brake release duration in milliseconds.
 */
#define BRAKE_RELEASE_DURATION          3000

/**
 * @brief Time to wait when updating the brake config in milliseconds.
 *        It is because the pwm service needs time to stop.
 */
#define BRAKE_UPDATE_CONFIG_WAIT        2000

/**
 * @brief Size of circular buffer for error items accumulating.
 */
#define ERROR_BUF_SIZE                  300

#define N_MOTION_CONTROL_INTERFACES     3



/**
 * @brief Position/Velocity control strategie
 */
typedef enum {
    POS_PID_CONTROLLER                      = 1,
    POS_PID_VELOCITY_CASCADED_CONTROLLER    = 2,
    LT_POSITION_CONTROLLER                  = 3,
    POS_PID_GAIN_SCHEDULING_CONTROLLER      = 4,
    VELOCITY_PID_CONTROLLER                 = 5
} MotionControlStrategies;

/**
 * @brief Brake control commands
 */
typedef enum {
  DISABLE_BRAKE = 0,
  ENABLE_BRAKE = 1
} BrakeCommands;

/**
 * @brief Motion polarity
 *
 *  When set to INVERTED (1) the position/velocity/torque commands will be inverted.
 *  The position/velocity/torque feedback is also inverted to match the commands.
 *  The position limits are also inverted to match the inverted position commands.
 *  The internal position of the controller is not changed, only the feedback.
 */
typedef enum {
    MOTION_POLARITY_NORMAL      = 0,
    MOTION_POLARITY_INVERTED    = 1
} MotionPolarity;


typedef enum {
    ERR_STATUS = 1,
    ERR_MOTION,
    ERR_SENSOR,
    ERR_SEC_SENSOR,
    ERR_ANGLE_SENSOR,
    ERR_WATCHDOG

} ErrType;

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
 * @brief Structure definition for a Motion Control Loop Service configuration.
 */
typedef struct {

    int position_control_strategy;      /**< Parameter for selecting between defferent types of position or velocity controllers. */
    int motion_profile_type;            /**< Parameter for selecting between different types of profilers (including torque/velocity/posiiton controllers. */

    int min_pos_range_limit;            /**< Parameter for setting the minimum position range */
    int max_pos_range_limit;            /**< Parameter for setting the maximum position range */
    int max_motor_speed;                /**< Parameter for setting the maximum motor speed */
    int max_torque;                     /**< Parameter for setting the maximum torque command which will be sent to torque controller */

    int enable_profiler;                /**< Parameter for enabling/disabling the profiler */
    int max_acceleration_profiler;      /**< Parameter for setting the maximum acceleration in profiler mode */
    int max_deceleration_profiler;      /**< Parameter for setting the maximum deceleration in profiler mode */
    int max_speed_profiler;             /**< Parameter for setting the maximum speed in profiler mode */
    int max_torque_rate_profiler;       /**< Parameter for setting the maximum torque in profiler mode */

    int torque_kp;                    /**< Parameter for torque controller P-constant */
    int torque_ki;                    /**< Parameter for torque controller I-constant */
    int torque_kd;                    /**< Parameter for torque controller D-constant */
    float position_kp;                    /**< Parameter for position controller P-constant */
    float position_ki;                    /**< Parameter for position controller I-constant */
    float position_kd;                    /**< Parameter for position controller D-constant */
    int position_integral_limit;        /**< Parameter for integral limit of position pid controller */

    int position_control_autotune;      /**< Parameter for enabling/disabling automatic tuning of position controller*/
    int step_amplitude_autotune;        /**< The tuning procedure uses steps to evaluate the response of controller. This input is equal to half of step command amplitude.*/
    int counter_max_autotune;           /**< The period of step commands in ticks. Each tick is corresponding to one execution sycle of motion_control_service. As a result, 3000 ticks when the frequency of motion_control_service is 1 ms leads to a period equal to 3 seconds for each step command.*/
    int per_thousand_overshoot_autotune;      /**< Overshoot limit while tuning (it is set as per thousand of step amplitude)*/
    int rise_time_freedom_percent_autotune;   /**< This value helps the tuner to find out whether the ki is high enough or not. By default set this value to 300, and if the tuner is not able to find proper values (and the response is having oscillations), increase this value to 400 or 500.*/

    float velocity_kp;                    /**< Parameter for velocity controller P-constant */
    float velocity_ki;                    /**< Parameter for velocity controller I-constant */
    float velocity_kd;                    /**< Parameter for velocity controller D-constant */
    int velocity_integral_limit;        /**< Parameter for integral limit of velocity pid controller */

    int enable_velocity_auto_tuner;     /**< Parameter for enabling/disabling auto tuner for velocity controller */
    int enable_compensation_recording;  /**< Parameter for enabling/disabling the cogging torque compensator recording*/
    int enable_open_phase_detection;    /**< Parameter for enabling/disabling the open phase detection functionality*/

    int k_fb;                           /**< Parameter for setting the feedback position sensor gain */
    int resolution;                     /**< Parameter for setting the resolution of position sensor [ticks/rotation] */
    int k_m;                            /**< Parameter for setting the gain of torque actuator */
    int moment_of_inertia;              /**< Parameter for setting the moment of inertia */
    MotionPolarity polarity;            /**< Parameter for setting the polarity of the movement */
    int brake_release_strategy;         /**< Parameter for setting different brake release strategies, e.g., shaking */
    int brake_release_delay;            /**< Parameter for setting the delay between removing voltage from the brake and disabling the control  */

    int dc_bus_voltage;                 /**< Parameter for setting the nominal (rated) value of dc-link */
    int pull_brake_voltage;             /**< Parameter for setting the voltage for pulling the brake out! */
    int pull_brake_time;                /**< Parameter for setting the time of brake pulling */
    int hold_brake_voltage;             /**< Parameter for setting the brake voltage after it is pulled */

    int filter;

    float position_kp_l;
    float position_ki_l;
    float position_kd_l;
    float position_kp_h;
    float position_ki_h;
    float position_kd_h;
    float velocity_kp_l;
    float velocity_ki_l;
    float velocity_kd_l;
    float velocity_kp_h;
    float velocity_ki_h;
    float velocity_kd_h;
    int velocity_lo_l;
    int velocity_hi_l;
} MotionControlConfig;

/**
 * @brief Structure definition for timestamp
 */
typedef struct {
    uint8_t mSec;
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} Timestamp_t;

/**
 * @brief Structure definition for one error item
 */
typedef struct {
    unsigned int index;
    Timestamp_t timestamp;
    unsigned int err_code;
    ErrType err_type;
} ErrItem_t;

/**
 * @brief Structure definition for a circular buffer for error items accumulating
 */
typedef struct {
    ErrItem_t buffer[ERROR_BUF_SIZE];
    int head;
    int tail;
} ErrBuf_t;


/**
 * @brief Interface type to communicate with the Motion Control Service.
 */
interface MotionControlInterface
{

    /**
     * @brief disables the motion control service
     */
    void disable();

    /**
     * @brief enables the position controler
     *
     * @param mode-> position control mode
     */
    void enable_position_ctrl(int mode);

    /**
     * @brief enables the velocity controller
     */
    void enable_velocity_ctrl(void);

    /**
     * @brief sets the moment of inertia of the load
     *
     * @param j-> moment of intertia
     */
    void set_j(int j);

    /**
     * @brief enables the torque controller
     */
    void enable_torque_ctrl();

    /**
     * @brief sets the reference value of torque in torque control mode
     *
     * @param target_torque -> torque reference in mNm
     */
    void set_torque(int target_torque);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    MotionControlConfig get_motion_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_motion_control_config(MotionControlConfig in_config);

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
     *
     * @param brake_status -> release if 1, block if 0
     */
    void set_brake_status(int brake_status);

    /**
     * @brief updates the new brake configuration in pwm service
     */
    void update_brake_configuration();

    /**
     * @brief Enables the offset detection process
     */
    MotorcontrolConfig set_offset_detection_enabled();

    /**
     * @brief Send a reset fault command to the motorcontrol
     */
    void reset_motorcontrol_faults();

    /**
     * @brief Enables the safe-torque-off mode
     *
     * @return void
     */
    void set_safe_torque_off_enabled();


    /**
     * @brief getter of actual position
     */
    int get_position();

    /**
     * @brief getter of actual velocity
     */
    int get_velocity();

    /**
     * @brief return the error for the open phase (0 = no error, 1 = phase A, 2 = phase B, 3 = phase C) and the value of phase resistance, if there was no error detected
     */
    {int, float} open_phase_detection();

    /**
     * @brief Enables the sensors evaluation
     *
     * @return sensor status (0 = no error, != 0 error)
     */
    int sensors_evaluation();

    /**
     * @brief responsible for data communication between torque controller and higher level controllers
     *
     * @param downstreamcontroldata -> structure including the commands for torque/velocity/position controller
     *
     * @return structure of type UpstreamControlData -> structure including the actual parameters (measurements, ...) from torque controller to higher controlling levels
     */
    UpstreamControlData update_control_data(DownstreamControlData downstreamcontroldata);

    /**
     * @brief           Enables/disables the cogging torque compensation
     *
     * @param   flag    value : 0 (disable) or 1 (enable)
     */
    void enable_cogging_compensation(int flag);

    /**
     * @brief    Pop last error item from error buffer
     *
     * @param    Error item
     */
    [[clears_notification]] int get_last_error(ErrItem_t &ErrItem);

    /**
     * @brief    Notification of new error
     */
    [[notification]] slave void new_error(void);
};


/**
 * @brief Initializer helper for the Position Control Service.
 *        It is required the client to call this function before
 *        starting to perform position control.
 *
 * @param i_motion_control Communication interface to the Position Control Service.
 *
 * @return void
 */
void init_motion_control(interface MotionControlInterface client i_motion_control);


/**
 * @brief Update brake hold/pull voltages and pull time in the pwm service.
 *
 *        It take the DC, hold/pull voltages and pull time parameters
 *        and compute the corresponding duty cycles which are then sent to the pwm service.
 *
 * @param motion_ctrl_config config structure of the motion control
 * @param i_torque_control client interface to get the IF2 tile frequency from the motorcontrol service.
 * @param i_update_brake client enterface to the pwm service to send the brake configuration
 *
 */
void update_brake_configuration(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control);


/**
 * @brief Enable motorcontrol and brake
 *
 *        The brake_release_strategy parameter it checked to chose the brake mode (disable, normal, shaking)
 *
 * @param motion_ctrl_config config structure of the motion control
 * @param i_torque_control client interface to enable motorcontrol and set the brake.
 * @param position used for the starting position for the shaking brake release
 * @param special_brake_release_counter used for the shaking brake release
 * @param special_brake_release_initial_position used for the shaking brake release
 * @param special_brake_release_torque used for the shaking brake release
 * @param motion_control_error error code of the motion control service
 *
 */
void enable_motorcontrol(MotionControlConfig &motion_ctrl_config, client interface TorqueControlInterface i_torque_control, int position,
        int &special_brake_release_counter, int &special_brake_release_initial_position, int &special_brake_release_torque, MotionControlError &motion_control_error);


/**
 * @brief Service to perform torque, velocity or position control.
 *        You will need a Motor Control Stack running parallel to this Service,
 *        have a look at Motor Control Service for more information.
 *        Service monitors if the open circuit exists in phases of the motor by observing the behaviour of phase currents.
 *        If the error exists, fault code for phase failure is generated.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param pos_velocity_control_config   Configuration for ttorque/velocity/position controllers.
 * @param i_torque_control Communication  interface to the Motor Control Service.
 * @param i_motion_control[3]         array of MotionControlInterfaces to communicate with upto 3 clients
 * @param i_update_brake                Interface to update brake configuration in PWM service
 *
 * @return void
 *  */
void motion_control_service(MotionControlConfig &pos_velocity_control_config,
                    interface TorqueControlInterface client i_torque_control,
                    interface MotionControlInterface server i_motion_control[3],
                    client interface UpdateBrake i_update_brake,
                    client interface i2c_master_if i2c);
