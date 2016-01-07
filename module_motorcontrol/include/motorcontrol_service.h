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
 * @brief Type for motors.
 */
typedef enum {
    BDC_MOTOR = 10,  /**< Brushed DC Motor. */
    BLDC_MOTOR = 11  /**< Brushless DC Motor. */
} MotorType;

/**
 * Structure type for Motorcontrol Service configuration.
 */
typedef struct {
    MotorType motor_type;               /**< Type of motor to drive. */
    BLDCWindingType bldc_winding_type;  /**< Type of winding of your motor (if using a BLDC motor). */
    int commutation_sensor;             /**< Absolute position sensor used for commutation (if using a BLDC motor).
                                             For the moment just Hall sensor can be used [HALL_SENSOR]. */
    int hall_offset[2];                 /**< Feedback Hall sensor offset for positive (hall_offset[0])
                                             and negative (hall_offset[1]) turning [0:4095].
                                             (Often required to optimize commutation if using a BLDC motor). */
    int commutation_loop_period;        /**< Period for the commutation loop [microseconds]. */
} MotorcontrolConfig;

#ifdef __XC__

#include <watchdog_service.h>
#include <hall_service.h>
#include <qei_service.h>

#include <internal_config.h>

/**
 * @brief Structure type to define the ports for the fets-driver in your IFM SOMANET device (if applicable).
 */
typedef struct {
    port ?p_coast;  /**< Port for management signals. */
    out port ?p_esf_rst_pwml_pwmh; /**< Port for management signals. */
    port ?p_ff1; /**< Port to read out faults. */
    port ?p_ff2; /**< Port to read out faults. */
} FetDriverPorts;

/**
 * @brief Lorem ipsum...
 */
interface MotorcontrolInterface{

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
    int check_busy();

    /**
     * @brief Lorem ipsum...
     *
     * @param voltage Lorem ipsum...
     */
    void set_voltage(int voltage);

    /**
     * @brief Lorem ipsum...
     *
     * @param parameters Lorem ipsum...
     */
    void set_parameters(MotorcontrolConfig parameters);

    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    MotorcontrolConfig get_config();

    /**
     * @brief Lorem ipsum...
     *
     * @param sensor Lorem ipsum...
     */
    void set_sensor(int sensor);

     /**
     * @brief Lorem ipsum...
     */
    void disable_fets();

    /**
     * @brief Lorem ipsum...
     */
    void enable_fets();

    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_fets_state();

    /**
     * @brief Lorem ipsum...
     *
     * @param hall_config Lorem ipsum...
     * @param qei_config Lorem ipsum...
     * @param commutation_config Lorem ipsum...
     * @param in_nominal_speed Lorem ipsum...
     */
    void set_all_parameters(HallConfig hall_config, QEIConfig qei_config, MotorcontrolConfig commutation_config);
};

/**
 * @brief Sinusoidal based Commutation Loop
 *
 * @param fet_driver_ports Lorem ipsum...
 * @param motorcontrol_config Lorem ipsum...
 * @param c_pwm_ctrl channel to set PWM level output to motor phases
 * @param i_hall Lorem ipsum...
 * @param i_qei Lorem ipsum...
 * @param i_watchdog Lorem ipsum...
 * @param i_motorcontrol[5] Lorem ipsum...
 *
 */
[[combinable]]
void motorcontrol_service(FetDriverPorts &fet_driver_ports, MotorcontrolConfig &motorcontrol_config,
                            chanend c_pwm_ctrl,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface WatchdogInterface client i_watchdog,
                            interface MotorcontrolInterface server i_motorcontrol[5]);

#endif
