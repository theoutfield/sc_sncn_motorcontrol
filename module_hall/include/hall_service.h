/**
 * @file hall_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**< Definition for referring to the Hall sensor */
#define HALL_SENSOR             1

#define ERROR                   0
#define SUCCESS                 1

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16
#define PULL_PERIOD_USEC 12


#define HALL_TICKS_PER_ELECTRICAL_ROTATION 4096

/**
 * Structure type for Hall Service configuration
 */
typedef struct {
    int pole_pairs; /**< Number of pole pairs in your motor */
} HallConfig;


#ifdef __XC__

/**
* Structure type for Hall Service ports
*/
typedef struct {
    port p_hall; /**< 4-bit port for Hall Sensor signals input */
} HallPorts;

/**
 * @brief Interface type to communicate with the Hall Service.
 */
interface HallInterface {
    /**
     * @brief Getter for the current pinstate at the Hall port.
     *
     * @return UVW signals state.
     */
    unsigned get_hall_pinstate();

    /**
     * @brief Getter for current position within one electrical rotation.
     *
     * @return Position within one electrical rotation [0:4095].
     *         Electrical rotation = Mechanical rotation/Pole-pairs.
     */
    int get_hall_position();

    /**
     * @brief Getter for calculated velocity of the motor.
     *
     * @return Mechanical RPMs.
     */
    int get_hall_velocity();

    /**
     * @brief Getter for absolute position.
     *
     * @return Absolute position [INT_MIN:INT_MAX].
     *         One electrical rotation = 4096 units.
     */
    int get_hall_position_absolute();

    /**
     * @brief Getter for direction of last change in the Hall Sensor signals.
     *
     * @return 1 for CW or positive rotation.
     *        -1 for CCW or negative rotation.
     */
    int get_hall_direction();

    /**
     * @brief Setter for the absolute position.
     *
     * @param offset New value for absolute position.
     */
    void reset_hall_absolute_position(int offset);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    HallConfig get_hall_config();

    /**
     * @brief Setter for the configuration used by the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_hall_config(HallConfig in_config);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 if not initialized.
     *         1 if initialized.
     */
    int check_busy();
};

/**
 *
 * @brief Service to read and process data from a Feedback Hall Sensor.
 *
 * @param hall_ports Port to get the Hall signals from.
 * @param hall_config Configuration for the service.
 * @param i_hall[5] \verbatim Array of communication interfaces to handle up to 5 different clients. \endverbatim
 */
[[combinable]]
void hall_service(HallPorts & hall_ports, HallConfig & hall_config,
                    interface HallInterface server i_hall[5]);

#endif
