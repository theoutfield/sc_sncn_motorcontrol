/**
 * @file motorcontrol_service.h
 * @brief Commutation Loop based on sinusoidal commutation method
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ERROR 0
#define SUCCESS 1

/**
 * @brief Type for the kind of winding of a BLDC motor.
 */
typedef enum {
    STAR_WINDING=1, /**< Star winding. */
    DELTA_WINDING   /**< Delta winding. */
} BLDCWindingType;

/**
 * @brief Type for the polarity of a motor.
 */
typedef enum {
    NORMAL_POLARITY=1, /**< Normal polarity. */
    INVERTED_POLARITY   /**< Inverted polarity. */
} PolarityType;

/**
 * @brief Type for motors.
 */
typedef enum {
    BDC_MOTOR = 10,  /**< Brushed DC Motor. */
    BLDC_MOTOR = 11  /**< Brush-less DC Motor. */
} MotorType;

/**
 * @brief Commutation method.
 */
typedef enum {
    SINE = 20,  /**< Sine commutation. */
    FOC = 21  /**< Vector control. */
} CommutationMethod;

/**
 * Structure type for Motorcontrol Service configuration.
 */
typedef struct {
    MotorType motor_type;                   /**< Type of motor to drive. */
    CommutationMethod commutation_method;   /**< Commutation method. */
    BLDCWindingType bldc_winding_type;      /**< Type of winding of your motor (if using a BLDC motor). */
    PolarityType polarity_type;             /**< Type of polarity of your motor. */
    int commutation_sensor;                 /**< Absolute position sensor used for commutation (if using a BLDC motor). For the moment just Hall sensor can be used [HALL_SENSOR]. */
    int hall_offset[2];                     /**< Feedback Hall sensor error offset for positive (hall_offset[0]) and negative (hall_offset[1]) turning [0:4095]. (Often required to optimize commutation if using a BLDC motor). */
    int commutation_loop_period;            /**< Period for the commutation loop [microseconds]. */
} MotorcontrolConfig;

#ifdef __XC__

#include <watchdog_service.h>
#include <adc_service.h>
#include <pwm_service.h>
#include <hall_service.h>
#include <memory_manager.h>

#include <mc_internal_constants.h>

/**
 * @brief Structure type to define the ports to manage the FET-driver in your IFM SOMANET device (if applicable).
 */
typedef struct {
    port ?p_coast;  /**< [Nullable] Port for management signals. */
    out port ?p_esf_rst_pwml_pwmh; /**< [Nullable] 4-bit Port to  enabling operation signals (if applicable in your SOMANET device). */
    port ?p_ff1; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
    port ?p_ff2; /**< [Nullable] Port to read out faults (if applicable in your SOMANET device). */
} FetDriverPorts;

/**
 * @brief Interface type to communicate with the Motor Control Service.
 */
interface MotorcontrolInterface{

    /**
     * @brief Notifies the interested parties that a new notification
     * is available.
     */
    [[notification]]
    slave void notification();

    /**
     * @brief Provides the type of notification currently available.
     *
     * @return type of the notification
     */
    [[clears_notification]]
    int get_notification();

    /**
     * @brief Enables the torque control
     */
    void set_torque_control_enabled();

    /**
     * @brief Disables the torque control
     */
    void set_torque_control_disabled();

    /**
     * @brief Enables the offset detection process
     */
    void set_offset_detection_enabled();

    /**
     * @brief Sets offset value
     */
    void set_offset_value(int offset_value);

    /**
     * @brief Sets an amplitude voltage on the sinusodial signals commutating the windings or Q value when FOC is used.
     *
     * @param voltage Voltage [-PWM_MAX_VALUE:PWM_MAX_VALUE]. By default PWM_MAX_VALUE = 13889. In case of FOC [-4096:4096]
     */
    void set_voltage(int voltage);

    /**
     * @brief Sets torque target value when FOC is used.
     *
     * @param torque_sp Torque [-4096:4096].
     */
    void set_torque(int torque_sp);

    /**
     * @brief Sets maximum torque control value when FOC is used.
     *
     * @param torque_sp Torque [-4096:4096].
     */
    void set_torque_max(int torque_sp);

    /**
     * @brief Setter for the configuration used by the Service.
     *        Note that not all configuration parameters can be changed on runtime.
     *
     * @param in_config New Service configuration.
     */
    void set_config(MotorcontrolConfig in_config);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    MotorcontrolConfig get_config();

    /**
     * @brief Setter for the status of the FETs
     *
     * @return 0 - FETs disabled.
     *         1 - FETs enabled.
     */
    void set_fets_state(int state);

    /**
     * @brief Getter for the status of the FETs
     *
     * @return 0 - FETs disabled.
     *         1 - FETs enabled.
     */
    int get_fets_state();

    /**
     * @brief Getter for actual torque.
     *
     * @return Torque actual.
     */
    int get_torque_actual();

    /**
     * @brief Getter for actual velocity.
     *
     * @return Velocity actual.
     */
    int get_velocity_actual();

    /**
     * @brief Getter for actual position.
     *
     * @return Position actual.
     */
    int get_position_actual();

    /**
     * @brief Allows you to change the commutation sensor on runtime.
     *
     * @param sensor New sensor [HALL_SENSOR]. (So far, just Hall sensor is available for commutation)
     */
    void set_sensor(int sensor);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized, 1 - initialized.
     */
    int check_busy();

    /**
     * @brief Set calib flag in the Motorcontrol service so it will alway set 0 as electrical angle
     *
     * @param flag 1 to activate, 0 to deactivate calibration
     */
    int set_calib(int flag);

    /**
     * @brief Set the sensor offset of the current position sensor
     *
     * @param Sensor offset
     */
    int set_sensor_offset(int in_offset);

    void set_control(int flag);

    {int, int, int} set_torque_pid(int Kp, int Ki, int Kd);

    void restart_watchdog();

    int get_field();
};


/**
 * @brief Service to drive BLDC and Brushed DC Motors.
 *        You will need additionally to have a PWM and Watchdog Services running.
 *
 *        If you are driving BLDC motors, also a parallel Service to read the
 *        absolute position of your rotor is required, for the moment
 *        just Hall Service is suitable for commutation purposes.
 *
 * @param fet_driver_ports Ports structure defining where to access the FET-driver signals.
 * @param motorcontrol_config Configuration for the Service.
 * @param c_pwm_ctrl Channel to PWM Service.
 * @param i_watchdog Interface to Watchdog Service.
 * @param i_motorcontrol Array of communication interfaces to handle up to 5 different clients.
 */
//[[combinable]]
void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            chanend c_pwm_ctrl,
                            interface ADCInterface client ?i_adc,
                            client interface shared_memory_interface ?i_shared_memory,
                            interface WatchdogInterface client i_watchdog,
                            interface BrakeInterface client ?i_brake,
                            interface MotorcontrolInterface server i_motorcontrol[4]);

#endif
