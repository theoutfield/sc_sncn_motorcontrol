/**
 * @file position_feedback_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <biss_config.h>
#include <rem_16mt_config.h>
#include <rem_14_config.h>

#include <biss_struct.h>
#include <rem_16mt_struct.h>
#include <rem_14_struct.h>
#include <hall_struct.h>
#include <qei_struct.h>
#include <motor_control_structures.h>
#include <refclk.h>

#include <stdint.h>

#define NUMBER_OF_GPIO_PORTS   4    /**< Defines number of Digital IOs available. */


/**
 * @brief GPIO port type
 */
typedef enum {
    GPIO_INPUT=0,           /**< Input GPIO port */
    GPIO_INPUT_PULLDOWN,    /**< Input GPIO port with pulldown */
    GPIO_OUTPUT             /**< Output GPIO port */
} GPIOType;


/**
 * @brief Sensor function type to select which data to write to the shared memory
 */
typedef enum {
    SENSOR_FUNCTION_DISABLED=0,                     /**< Send nothing */
    SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL, /**< Send the electrical angle for commutation and the absolute position/velocity for motion control */
    SENSOR_FUNCTION_COMMUTATION_AND_FEEDBACK_ONLY,  /**< Send the electrical angle for commutation and the absolute position/velocity for secondary feedback (for display only) */
    SENSOR_FUNCTION_MOTION_CONTROL,                 /**< Send only the absolute position/velocity for motion control */
    SENSOR_FUNCTION_FEEDBACK_ONLY                   /**< Send only the absolute position/velocity for secondary feedback (for display only) */
} SensorFunction;


/**
 * @brief Type for sensor polarity
 *
 *        When set to inverted it will reverse the position, velocity and electrical angle direction.
 *        It the same effect as changing the sensor placement from back to front or the other way around.
 *        Depending on the sensor the change is made in software (BiSS, Hall, QEI)
 *        or by setting a register in the sensor (REM 16MT, REM 14)
 */
typedef enum {
    SENSOR_POLARITY_NORMAL  = 1,    /**< Normal polarity. */
    SENSOR_POLARITY_INVERTED=-1     /**< Inverted polarity. */
} SensorPolarity;



/**
 * @brief Configuration structure of the position feedback service.
 */
typedef struct {
    SensorType sensor_type;         /**< Select the sensor type */
    SensorFunction sensor_function; /**< Select which data to write to shared memory */
    SensorPolarity polarity;        /**< Encoder polarity. */
    UsecType ifm_usec;              /**< Number of clock ticks in a microsecond >*/
    int pole_pairs;                 /**< Number of pole pairs */
    int resolution;                 /**< Number of ticks per turn */
    int offset;                     /**< Position offset in ticks, can be singleturn or multiturn depending on the sensor */
    int max_ticks;                  /**< The multiturn position is reset to 0 when reached */
    int velocity_compute_period;    /**< Velocity compute period in microsecond. Is also the polling period to write to the shared memory */
    BISSConfig biss_config;         /**< BiSS sensor configuration */
    REM_16MTConfig rem_16mt_config; /**< REM 16MT sensor configuration */
    REM_14Config rem_14_config;     /**< REM 14  configuration */
    QEIConfig qei_config;           /**< QEI sensor configuration */
    HallConfig hall_config;         /**< Hall sensor configuration */
    GPIOType gpio_config[4];        /**< GPIO configuration */
} PositionFeedbackConfig;


#ifdef __XC__

#include <spi_master.h>


/**
 * @brief Interface to communicate with the Position Feedback Service.
 */
interface PositionFeedbackInterface
{
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
     * @brief Get the electrical angle
     *
     * @return electrical angle
     */
    unsigned int get_angle(void);

    /**
     * @brief Get the absolute multiturn position, the singleturn position and the sensor status
     *
     * @return Absolute multiturn position in ticks
     * @return Singleturn position in ticks
     * @return Sensor status
     */
    { int, unsigned int, unsigned int } get_position(void);

    /**
     * @brief Get the velocity in Round per minute (rpm)
     *
     * @return velocity in rpm
     */
    int get_velocity(void);

    /**
     * @brief Get the position feedback configuration
     *
     * @return position feedback configuration
     */
    PositionFeedbackConfig get_config(void);

    /**
     * @briefSet the position feedback configuration
     *
     * @param in_config the position feedback configuration to set
     *
     */
    void set_config(PositionFeedbackConfig in_config);

    /**
     * @brief Reset the absolute position to a new value
     *
     * @param the new value to set
     */
    void set_position(int in_count);

    /**
     * @brief Send a command to the sensor (currently only supported for REM 16MT)
     *
     * @param opcode of the command
     * @param data of the command
     * @param data_bits the number of bits of data
     *
     * @return return status of the command
     */
    unsigned int send_command(int opcode, int data, int data_bits);

    /**
     * @brief Read a GPIO port
     *
     * @param gpio_num The number of GPIO port to read
     *
     * @return Value read
     */
    int gpio_read(int gpio_num);

    /**
     * @brief Write to a GPIO port
     *
     * @param gpio_num The number of the GPIO port
     * @param in_value Value to write
     */
    void gpio_write(int gpio_num, int in_value);

    /**
     * @brief Exit the current sensor service and restart the position
     *
     */
    void exit();
};


/**
 * @brief Structure for SPI ports and clock blocks
 */
typedef struct {
    spi_master_interface spi_interface;
    port * movable slave_select;
} SPIPorts;

/**
 * @brief Structure for Hall/QEI input ports (can also be used for BiSS)
 */
typedef struct {
    in port p_qei_hall; /**< 4-bit Port for Encoder, BiSS or Hall signals input. */
} QEIHallPort;

/**
 * @brief Structure for the hall_enc_select port used to select the mode (differential or not) of Hall/qei ports. Also used for the BiSS clock output
 */
typedef struct {
    out port ?p_hall_enc_select; /**< [Nullable] Port to control the signal input circuitry (if applicable in your SOMANET device). Also used for the BiSS clock output */
} HallEncSelectPort;

#include <shared_memory.h>
#include <biss_service.h>
#include <rem_16mt_service.h>
#include <rem_14_service.h>
#include <hall_service.h>
#include <qei_service.h>


/**
 * @brief Service to read and position, velocity and electrical angle from various position sensors (Hall, QEI, BiSS, REM 16MT, REM 14)
 *        It can also manages the GPIO ports.
 *
 * The service can simultaneously run 2 sensor services and manage the GPIO ports.
 * Is uses the shared memory to send position, velocity and electrical angle to the other services.
 *
 * @param qei_hall_port_1 Hall/QEI input port number 1
 * @param qei_hall_port_2 Hall/QEI input port number 2
 * @param hall_enc_select_port port used to select the mode (differential or not) of Hall/qei ports
 * @param spi_ports SPI ports and clock blocks
 * @param gpio_port_0 GPIO port number 0
 * @param gpio_port_1 GPIO port number 1
 * @param gpio_port_2 GPIO port number 2
 * @param gpio_port_3 GPIO port number 3
 * @param position_feedback_config_1 Config structure for first service
 * @param i_shared_memory_1 Shared memory interface for first service
 * @param i_position_feedback_1 Server interface for first service
 * @param position_feedback_config_2 Config structure for second service
 * @param i_shared_memory_2 Shared memory interface for second service
 * @param i_position_feedback_2 Server interface for second service
 *
 */
void position_feedback_service(QEIHallPort &?qei_hall_port_1, QEIHallPort &?qei_hall_port_2, HallEncSelectPort &?hall_enc_select_port, SPIPorts &?spi_ports, port ?gpio_port_0, port ?gpio_port_1, port ?gpio_port_2, port ?gpio_port_3,
                               PositionFeedbackConfig &position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3]);

/**
 * @brief Convert the number of tick per turn into the number of resolution bits
 *
 * @param ticks the number of ticks per turn
 *
 * @return the number of resolution bits
 *
 */
int tickstobits(uint32_t ticks);

/**
 * @brief Compute the multiturn position using the last measured position and the current position
 *
 * @param count The multiturn position
 * @param last_position The last measured position
 * @param position The current position
 * @param ticks_per_turn The number of ticks per turn
 */
void multiturn(int &count, int last_position, int position, int ticks_per_turn);


/**
 * @brief Write position, velocity and electrical angle to the shared memory.
 *
 * @param i_shared_memory The client interface to the shared memory
 * @param sensor_function The sensor function to select which data to write.
 * @param count The absolute multiturn position
 * @param velocity The velocity
 * @param angle The electrical angle
 * @param hall_state The Hall pin state if Hall sensor is used
 */
void write_shared_memory(client interface shared_memory_interface ?i_shared_memory, SensorFunction sensor_function, int count, int velocity, int angle, int hall_state);

/**
 * @brief Compute the velocity in rpm
 *
 * @param difference The difference in ticks between the 2 last measurements
 * @param timediff The time difference between the 2 last measurements
 * @param resolution The number of ticks per turn (to convert ticks to rpm)
 *
 * @return velocity in rpm
 */
int velocity_compute(int difference, int timediff, int resolution);

/**
 * @brief Read a GPIO port
 *
 * @param gpio_ports The GPIO ports array
 * @param position_feedback_config The position feedback service configuration
 * @param gpio_number The GPIO port number
 *
 * @return The value of the GPIO port
 */
int gpio_read(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number);

/**
 * @brief Write to a GPIO port
 *
 * @param gpio_ports The GPIO ports array
 * @param position_feedback_config The position feedback service configuration
 * @param gpio_number The GPIO port number
 * @param value The value to write
 */
void gpio_write(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number, int value);

/**
 * @brief Read/Write GPIO data from/to the shared memory.
 *
 *        The input data is read from the GPIO ports and written to the shared memory.
 *        The output data is read from the shared memory and written to the GPIO ports.
 *
 *
 * @param gpio_ports The GPIO ports array
 * @param position_feedback_config The position feedback service configuration
 * @param i_shared_memory The client interface to the shared memory
 */
void gpio_shared_memory(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory);


#endif
