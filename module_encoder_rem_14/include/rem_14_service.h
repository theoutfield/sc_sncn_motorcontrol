/*
 * rotary_sensor.h
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
void initRotarySensorInterface(SPIPorts &spi_ports);

/**
 * @brief Initialize REM 14 sensor
 *
 * @param spi_ports the SPI ports structure
 * @param config position feedback config containing the REM 14 sensor config
 *
 * @return status
 */
SensorError initRotarySensor(SPIPorts &spi_ports, PositionFeedbackConfig &config);


//reading fx
//non-volatile regs

/**
 * @brief Read the zero position (offset)
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return zero position (offset)
 */
int readZeroPosition(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the number of pole pairs
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return number of pole pairs
 */
int readNumberPolePairs(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the first part of the setting register
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return first part of the setting register
 */
int readSettings1(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the second part of the setting register
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return second part of the setting register
 */
int readSettings2(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the redundancy register
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return redundancy register
 */
int readRedundancyReg(SPIPorts &spi_ports, UsecType tile_usec);


//volatile regs

/**
 * @brief Read the programming register
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return programming register
 */
int readProgrammingReg(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the Cordic magnitude
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return Cordic magnitude
 */
int readCORDICMagnitude(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the Diagnostic and AutoGain Control
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return Diagnostic and AutoGain Control
 */
int readRotaryDiagnosticAndAutoGainControl(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the sensor error
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return sensor error
 */
int readRotarySensorError(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the singleturn position without compensation
 *
 * @paramspi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return singleturn position without compensation
 * @return status
 */
{ unsigned int, unsigned int } readRotarySensorAngleWithoutCompensation(SPIPorts &spi_ports, UsecType tile_usec);

/**
 * @brief Read the singleturn position with compensation
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 *
 * @return singleturn position with compensation
 * @return status
 */
{ unsigned int, unsigned int } readRotarySensorAngleWithCompensation(SPIPorts &spi_ports, UsecType tile_usec);


//writing fx

/**
 * @brief Write the setting register
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 * @param address the address of the register to write (settings1 or settings2)
 * @param data the data to write
 *
 * @return status
 */
int writeSettings(SPIPorts &spi_ports, UsecType tile_usec, unsigned short address, unsigned short data);

/**
 * @brief Write the zero position (offset)
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 * @param data the data to write
 *
 * @return status
 */
int writeZeroPosition(SPIPorts &spi_ports, UsecType tile_usec, unsigned short data);

/**
 * @brief Write the number of pole pairs
 *
 * @param spi_ports the SPI ports structure
 * @param tile_usec         number of ticks in a microseconds
 * @param data the data to write
 *
 * @return status
 */
int writeNumberPolePairs(SPIPorts &spi_ports, UsecType tile_usec, unsigned short data);

#endif
