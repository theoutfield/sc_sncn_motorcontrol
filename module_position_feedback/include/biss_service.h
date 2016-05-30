/**
 * @file biss_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>




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
#define BISS_CLOCK_DIVISOR         20           // supported frequencies are (tile frequency) / 2n
#define BISS_USEC                  USEC_FAST    // number of ticks in a microsecond
#define BISS_VELOCITY_LOOP         100         // velocity loop time in microseconds
#define BISS_TIMEOUT               15*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     4000



/**
 * @brief Type for the return status when reading BiSS data
 */
typedef enum {
    NoError=0,       /**< no error */
    CRCCorrected=1,  /**< CRC corrected  */
    CRCError=2,      /**< CRC mismatch  */
    NoAck,           /**< Ack bit not found. */
    NoStartBit       /**< Start bit not found */
} BISS_ErrorType;

#ifdef __XC__

#include <platform.h>
#include <memory_manager.h>
#include <position_feedback_service.h>



/**
 * @brief Service to read and process data from an Feedback BiSS Encoder Sensor.
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param biss_config Configuration for the service.
 * @param i_shared_memory Communication interface to the shared memory service.
 * @param i_biss Array of communication interfaces to handle up to 5 different clients.
 */
void biss_service(BISSPorts & biss_ports, BISSConfig & biss_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3]);


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
