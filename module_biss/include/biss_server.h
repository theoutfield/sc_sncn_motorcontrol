/**
 * @file biss_server.h
 * @author Synapticon GmbH <support@synapticon.com>
 */
#pragma once

#include <platform.h>
#include <biss_interface.h>


/**
 * @brief Initialize BuSS encoder parameters
 *
 * @param[out] biss_params struct defines the data lengths and the crc polynom of the encoder
 */
void init_biss_param(biss_par &biss_params);


/**
 * @brief Implementation of the BiSS encoder server thread
 *
 * @param[out] i_biss array of interfaces to send the data to the client
 * @param n number of client interfaces
 * @param p_biss_clk 1-bit out port to output the biss clock
 * @param p_biss_data in port for reading the biss encoder data
 * @param clk clock to generate the biss clock
 * @param biss_params structure definition for the biss encoder data lengths and crc polynom
 * @param frame_bytes number of 32 bit bytes to read from the encoder, should be able to contain at least 2 bits + ack and start bits + data + crc
 */
void run_biss(server interface i_biss i_biss[n], unsigned int n, port out p_biss_clk, port in p_biss_data, clock clk,
              biss_par & biss_params, static const int frame_bytes);


/**
 * @brief Read generic BiSS sensor data
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
 * @return error status (NoError, CRCError, NoAck, NoStartBit)
 */
unsigned int read_biss_sensor_data(port out p_biss_clk, port in p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly);


/**
 * @brief Extract turn data from a biss encoder sensor data
 *
 * @param data biss sensor data
 * @param biss_params structure definition for the biss encoder data lengths
 *
 * @return absolute count
 * @return position in the range [0 - (2^singleturn_resolution - 1)]
 * @return status bits (error and warning bits), 0 = ok
 */
{ int, unsigned int, unsigned int } biss_encoder(unsigned int data[], biss_par biss_params);


/**
 * @brief Compute a crc for biss data
 *
 * @param data biss data
 * @param data_length length of data in bits
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 *
 * @return inverted crc for biss
 */
unsigned int biss_crc(unsigned int data[], unsigned int data_length, unsigned int poly);


/**
 * @brief Try 1-bit error correction for biss data
 *
 * @param[out] data biss data
 * @param data_length length of data in bits
 * @param frame_bytes number of 32 bit bytes of data
 * @param crc_received crc received with the data
 * @param crc_poly crc polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100
 */
void biss_crc_correct(unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int poly);

unsigned int biss_position_fast(port out p_biss_clk, port in p_biss_data, clock clk, unsigned a, unsigned b, int multiturn_length, int singleturn_length);
