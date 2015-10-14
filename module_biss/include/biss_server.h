/**
 * @file biss_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */
#pragma once

#include <platform.h>
#include <biss_config.h>


/**
 * @brief Initialize biss encoder parameters
 *
 * @param[out] biss_params struct defines the data lengths and the crc polynom of the encoder
 */
void init_biss_param(biss_par &biss_params);


/**
 * @brief BiSS encoder server
 *
 * @param[out] i_biss array of interfaces to send the data to the client
 * @param p_biss_clk 1-bit out port to output the biss clock
 * @param p_biss_data in port for reading the biss encoder data
 * @param clk clock to generate the biss clock
 * @param a the dividend of the desired clock rate
 * @param b the divisor of the desired clock rates
 * @param biss_params structure definition for biss encoder
 * @param frame_bytes number of 32 bit bytes to read from the encoder, should be able to contain 2 bits + ack and start bits + data + crc
 */
void run_biss(server interface i_biss i_biss[2], port out p_biss_clk, port p_biss_data, clock clk, unsigned a, unsigned b,
              biss_par & biss_params, static const int frame_bytes);


/**
 * @brief Read generic biss sensor data
 *
 * @param p_biss_clk 1-bit out port to output the biss clock
 * @param p_biss_data in port for reading the biss encoder data
 * @param clk clock to generate the biss clock
 * @param a the dividend of the desired clock rate
 * @param b the divisor of the desired clock rates
 * @param[out] data array to store the received bits
 * @param data_length length of sensor data in bits (without crc)
 * @param frame_bytes number of 32 bit bytes to read from the encoder, should be able to contain 2 bits + ack and start bits + data + crc
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return error status
 */
unsigned int read_biss_sensor_data(port out p_biss_clk, port p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly);


/**
 * @brief Extract turn data from biss encoder sensor data
 *
 * @param data biss sensor data
 * @param multiturn_length length of multiturn data
 * @param singleturn_length length of singleturn data
 * @param status_length length of status data: error and warning bits
 *
 * @return absolute count, position, inverted status (error and warning bits)
 */
{ int, unsigned int, unsigned int } biss_encoder(unsigned int data, int multiturn_length, int singleturn_length, int status_length);


/**
 * @brief Compute a crc for biss data
 *
 * @param data biss data
 * @param data_length length of data in bits
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return inverted crc for biss
 */
unsigned int biss_crc(unsigned int *data, unsigned int data_length, unsigned int poly);


/**
 * @brief Try 1-bit error correction for biss data
 *
 * @param[out] data biss data
 * @param data_length length of data in bits
 * @param frame_bytes number of 32 bit bytes of data
 * @param crc_received crc received with the data
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 */
void biss_crc_correct(unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int poly);
