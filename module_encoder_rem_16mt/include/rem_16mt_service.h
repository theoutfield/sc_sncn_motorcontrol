/*
 * rem_16mt_service.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

/**
 * @brief Initialize SPI ports and clock blocks
 *
 * @param spi_ports the SPI ports structure
 */
void init_spi_ports(SPIPorts &spi_ports);


/**
 * @brief Initialize REM 16MT sensor
 *
 * @param spi_ports the SPI ports structure
 * @param config position feedback config containing the REM 16 sensor config
 *
 * @return status
 */
SensorError rem_16mt_init(SPIPorts &spi_ports, PositionFeedbackConfig &config);


/**
 * @brief REM 16MT sensor position data
 *
 * @param spi_ports the SPI ports structure
 * @param ifm_usec number of ticks in a microseconds
 *
 * @return status
 * @return absolute multiturn count
 * @return singleturn position filtered
 * @return singleturn position raw
 * @return timestamp
 */
{ SensorError, int, unsigned int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports, UsecType ifm_usec);


/**
 * @brief Write REM 16MT command
 *
 * @param spi_ports the SPI ports structure
 * @param opcode the opcode of the command
 * @param data the data to write
 * @param data_bits the number of data bits to write
 * @param ifm_usec number of ticks in a microseconds
 */
void rem_16mt_write(SPIPorts &spi_ports, int opcode, int data, int data_bits, UsecType ifm_usec);

#endif
