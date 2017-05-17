/**
 * @file biss_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>


/**
 * @brief Read generic BiSS sensor data
 *
 * @param qei_hall_port BiSS/SSI input port
 * @param hall_enc_select_port port used to select the mode (differential or not) of Hall/qei ports and optionally output the BiSS clock
 * @param hall_enc_select_config config to select the mode (differential or not) of Hall/qei ports
 * @param gpio_ports to optionally use gpio ports for input/output
 * @param t used for the timeout
 * @param position_feedback_config Configuration of the BiSS sensor (data lengths, crc polynomial, etc)
 * @param[out] data Array to store the read bits, should be large enough to store all the data bits + crc bits
 *
 * @return error status (No Error, CRC Error, No Ack, No Start Bit)
 */
SensorError read_biss_sensor_data(QEIHallPort * qei_hall_port,
        HallEncSelectPort * hall_enc_select_port, int hall_enc_select_config,
        port * (&?gpio_ports)[4], timer t,
        PositionFeedbackConfig &position_feedback_config, unsigned int data[]);

/**
 * @brief Read BiSS/SSI bit
 *
 * @param qei_hall_port BiSS/SSI input port
 * @param data_port port used to optionally input the BiSS/SSI data with a gpio port
 * @param data_port_config input port configuration
 *
 * @return bit read
 */
unsigned int read_biss_bit(QEIHallPort * qei_hall_port, port *data_port, int data_port_config);

/**
 * @brief Extract position data from a BiSS encoder raw sensor data
 *
 * @param data BiSS raw sensor data
 * @param position_feedback_config structure definition for the BiSS encoder data lengths
 *
 * @return absolute count
 * @return position in the range [0 - (2^singleturn_resolution - 1)]
 * @return status (error and warning bits)
 */
{ int, unsigned int, SensorError } biss_encoder(unsigned int data[], PositionFeedbackConfig &position_feedback_config);


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
