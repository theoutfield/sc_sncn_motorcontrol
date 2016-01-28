/**
 * @file qei_server.h
 * @brief QEI Sensor Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**
* @brief Definition for referring to the Encoder sensor.
*/
#define QEI_SENSOR               2

#define QEI_CHANGES_PER_TICK     4 //Quadrature encoder

#define ERROR                    0
#define SUCCESS                  1

#define FILTER_LENGTH_QEI        8
#define FILTER_LENGTH_QEI_PWM    8

#define QEI_RPM_CONST            1000*60
#define QEI_PWM_RPM_CONST        18000*60

#define QEI_RAW_POS_REQ          1
#define QEI_ABSOLUTE_POS_REQ     2
#define QEI_VELOCITY_REQ         3
#define QEI_VELOCITY_PWM_RES_REQ 4
#define SYNC                     5
#define SET_OFFSET               6
#define QEI_RESET_COUNT          7

#define QEI_POLARITY_NORMAL      1
#define QEI_POLARITY_INVERTED   -1

#define QEI_PORT_AS_TTL           0b0000
#define QEI_PORT_AS_RS422         0b0010


/**
 * @brief Type for the kind of Encoder output signals.
 */
typedef enum {
    QEI_RS422_SIGNAL = 11,  /**< Encoder signal output over RS422 (differential). */
    QEI_TTL_SIGNAL = 22     /**< Encoder signal output over standard TTL signal.  */
} QEI_SignalType;

/**
 * @brief Type for the sort of Encoder index.
 */
typedef enum {
    QEI_WITH_NO_INDEX = 3,  /**< Encoder with no index signal. */
    QEI_WITH_INDEX  = 4     /**< Encoder with index signal.  */
} QEI_IndexType;

/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    int ticks_resolution;       /**< Encoder resolution [pulses/revolution]. */
    QEI_IndexType index_type;   /**< Encoder index type. */
    int sensor_polarity;        /**< Encoder direction. */
    QEI_SignalType signal_type; /**< Encoder output signal type (if applicable in your SOMANET device). */
} QEIConfig;


#ifdef __XC__

/**
 * @brief Structure type to define the Encoder Service ports.
 */
typedef struct {
    port ?p_qei_config; /**< [Nullable] Port to control the signal input circuitry (if applicable in your SOMANET device). */
    port p_qei; /**< 4-bit Port for Encoder Interface signals input. */
} QEIPorts;

/**
 * @brief Interface type to communicate with the Encoder Service.
 */
interface QEIInterface{

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
     * @brief Getter for current position.
     *
     * @return  Position within one mechanical rotation [0: 4 x Encoder resolution].
     * @return  Validity of the returned position (for indexed Encoders).
     *          0 - not valid.
     *          1 - valid.
     */
    {unsigned int, unsigned int} get_qei_position();

    /**
     * @brief Getter for calculated velocity of the motor.
     *
     * @return Mechanical RPMs.
     */
    int get_qei_velocity();

    /**
     * @brief Getter for direction of last change in the Encoder signals.
     *
     * @return 1 for CW or positive rotation.
     *        -1 for CCW or negative rotation.
     */
    int get_qei_direction();

    /**
     * @brief Getter for current absolute position.
     *
     * @return Absolute counted up position [INT_MIN:INT_MAX].
     *         One mechanical rotation = 4 x Encoder resolution.
     */
    int get_qei_position_absolute();

    /**
     * @brief Setter for the absolute position.
     *
     * @param offset New value for absolute position.
     */
    void reset_qei_absolute_position(int offset);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    QEIConfig get_qei_config();

    /**
     * @brief Setter for the configuration used by the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_qei_config(QEIConfig in_config);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 if not initialized.
     *         1 if initialized.
     */
    int check_busy();

     // For internal use
    {int, int, int} get_qei_sync_position();

    // For internal use
    void set_qei_sync_offset(int, int);
};


/**
 * @brief Service to read and process data from an Feedback Incremental Encoder Sensor.
 *
 * @param qei_ports Ports structure defining where to access the Encoder signals.
 * @param qei_config Configuration for the service.
 * @param i_qei[5] Array of communication interfaces to handle up to 5 different clients.
 */
void qei_service(QEIPorts & qei_ports, QEIConfig qei_config,
                interface QEIInterface server i_qei[5]);

#endif
