/**
 * @file biss_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

/**
 * @brief Service to read and process data from an Feedback BiSS Encoder Sensor.
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param biss_config Configuration for the service.
 * @param i_shared_memory Communication interface to the shared memory service.
 * @param i_biss Array of communication interfaces to handle up to 5 different clients.
 */
void biss_service(BISSPorts &biss_ports, BISSConfig & biss_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3]);


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
unsigned int read_biss_sensor_data(BISSPorts &biss_ports, BISSConfig & biss_config, unsigned int data[], static const unsigned int frame_bytes);


/**
 * @brief Read up to 32 bit of BiSS sensor data without CRC checking
 *
 * @param biss_ports Ports structure defining where to read the BiSS signal, outputting the clock and which clock block to use.
 * @param before_length length of data to dismiss
 * @param data_length length of desired data
 *
 * @return data
 */
unsigned int read_biss_sensor_data_fast(BISSPorts &biss_ports, int before_length, int data_length);


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
