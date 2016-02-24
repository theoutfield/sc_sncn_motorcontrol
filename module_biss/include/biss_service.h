/**
 * @file biss_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>

/**
 * @brief Definition for referring to the BiSS sensor.
 */
#define BISS_SENSOR                4

#define ERROR                      0
#define SUCCESS                    1

#define SET_ALL_AS_QEI                 0b0011
#define SET_PORT1_AS_HALL_PORT2_AS_QEI 0b0010
#define SET_PORT1_AS_QEI_PORT2_AS_HALL 0b0001

#define BISS_POLARITY_NORMAL       0
#define BISS_POLARITY_INVERTED     1

#define BISS_MULTITURN_RESOLUTION  12
#define BISS_SINGLETURN_RESOLUTION 13
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION + 1 //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION
#define BISS_FRAME_BYTES           (( (3 + 2 + BISS_MULTITURN_LENGTH + BISS_SINGLETURN_LENGTH + BISS_STATUS_LENGTH + 6) -1)/32 + 1) //at least 3 bits + ack and start bits + data + crc
#define BISS_POLARITY              BISS_POLARITY_NORMAL
#define BISS_MAX_TICKS             0x7fffffff   // the count is reset to 0 if greater than this
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_DATA_PORT_BIT         0            // bit number (0 = rightmost bit) when inputing from a multibit port
#define BISS_CLK_PORT_HIGH         (0b1000 | SET_PORT1_AS_HALL_PORT2_AS_QEI)    // high clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLK_PORT_LOW          SET_PORT1_AS_HALL_PORT2_AS_QEI               // low  clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequency: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         22           // supported frequencies are (tile frequency) / 2n
#define BISS_USEC                  USEC_FAST    // number of ticks in a microsecond
#define BISS_VELOCITY_LOOP         1000         // velocity loop time in microseconds
#define BISS_TIMEOUT               14*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     4000

/**
 * @brief Structure type to define the BiSS Service configuration.
 */
typedef struct {
    int multiturn_length;       /**< Number of bits used for multiturn data */
    int multiturn_resolution;   /**< Number of bits of multiturn resolution */
    int singleturn_length;      /**< Number of bits used for singleturn data */
    int singleturn_resolution;  /**< Number of bits of singleturn resolution */
    int status_length;          /**< Rumber of bits used for status data */
    int crc_poly;               /**< CRC polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100 */
    int pole_pairs;             /**< Number of poles pairs to compute the electrical angle from the mechanical angle*/
    int polarity;               /**< Polarity, invert the direction */
    int clock_dividend;         /**< BiSS output clock frequency dividend */
    int clock_divisor;          /**< BiSS output clock frequency divisor */
    int timeout;                /**< Timeout after a BiSS read in clock ticks */
    int velocity_loop;          /**< Velocity loop time in microseconds */
    int max_ticks;              /**< The count is reset to 0 if greater than this */
    int offset_electrical;      /**< Offset for the electrical angle */
} BISSConfig;


/**
 * @brief Type for the return status when reading BiSS data
 */
typedef enum {
    NoError,    /**< no error */
    CRCError,   /**< CRC mismatch  */
    NoAck,      /**< Ack bit not found. */
    NoStartBit  /**< Start bit not found */
} BISS_ErrorType;

#ifdef __XC__

#include <platform.h>

/**
 * @brief Structure type to define the BiSS Service ports.
 */
typedef struct {
    port p_biss_data;   /**< Port for BiSS Interface signal input. */
    port p_biss_clk;    /**< Port for BiSS Interface clock output. */
    clock clk;          /**< Hardware clock used as time reference */
} BISSPorts;

/**
 * @brief Interface type to communicate with the BiSS Service.
 */
interface BISSInterface {

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
     * @brief Getter for absolute and singleturn position.
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_biss_position();

    /**
     * @brief Getter for real internal encoder position, not adjusted for polarity or offset.
     *
     * @return absolute position
     * @return singleturn position
     * @return error and warning bits from the encoder
     */
    { int, unsigned int, unsigned int } get_biss_real_position();

    /**
     * @brief Getter for only singleturn position without CRC checking.
     *
     * @return singleturn position
     */
    unsigned int get_biss_position_fast();

    /**
     * @brief Getter for electrical angle which is singleturn position * pole pairs.
     *
     * @return electrical angle
     */
    unsigned int get_biss_angle();

    /**
     * @brief Getter for calculated velocity.
     *
     * @return Mechanical RPMs.
     */
    int get_biss_velocity();

    /**
     * @brief Setter for the absolute position.
     *
     * @param new_count New value for absolute position.
     */
    void reset_biss_position(int new_count);

    /**
     * @brief Setter for electrical angle
     *
     * @param angle electrical angle to set
     *
     * @return new electrical angle offset
     */
    unsigned int reset_biss_angle_electrical(unsigned int angle);

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current configuration.
     */
    BISSConfig get_biss_config();

    /**
     * @brief Setter for the configuration used by the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_biss_config(BISSConfig in_config);
};


/**
 * @brief Service to read and process data from an Feedback BiSS Encoder Sensor.
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param biss_config Configuration for the service.
 * @param i_biss Array of communication interfaces to handle up to 5 different clients.
 */
void biss_service(BISSPorts & biss_ports, BISSConfig & biss_config, interface BISSInterface server i_biss[5]);


/**
 * @brief Read generic BiSS sensor data
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param biss_config data lengths and crc polynomial.
 * @param[out] data array to store the received bits
 * @param frame_bytes number of 32 bit bytes to read from the encoder, should be able to contain 3 bits + ack and start bits + data + crc
 *
 * @return error status (NoError, CRCError, NoAck, NoStartBit)
 */
unsigned int read_biss_sensor_data(BISSPorts & biss_ports, BISSConfig & biss_config, unsigned int data[], static const unsigned int frame_bytes);


/**
 * @brief Read up to 32 bit of BiSS sensor data without CRC checking
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param before_length length of data to dismiss
 * @param data_length length of desired data
 *
 * @return data
 */
unsigned int read_biss_sensor_data_fast(BISSPorts & biss_ports, int before_length, int data_length);


/**
 * @brief Extract turn data from a BiSS encoder sensor data
 *
 * @param data BiSS sensor data
 * @param biss_config structure definition for the BiSS encoder data lengths
 *
 * @return absolute count
 * @return position in the range [0 - (2^singleturn_resolution - 1)]
 * @return status bits (error and warning bits), 0 = ok
 */
{ int, unsigned int, unsigned int } biss_encoder(unsigned int data[], BISSConfig biss_config);


/**
 * @brief Compute a crc for BiSS data
 *
 * @param data BiSS data
 * @param data_length length of data in bits
 * @param crc_poly crc polynomial in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return inverted crc for BiSS
 */
unsigned int biss_crc(unsigned int data[], unsigned int data_length, unsigned int crc_poly);


/**
 * @brief Try 1-bit error correction for BiSS data
 *
 * @param[out] data BiSS data
 * @param data_length length of data in bits
 * @param frame_bytes number of 32 bit bytes of data
 * @param crc_received crc received with the data
 * @param crc_poly crc polynomial in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 */
void biss_crc_correct(unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int crc_poly);

#endif
