/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

void initRotarySensorInterface(SPIPorts &spi_ports);
int initRotarySensor(SPIPorts &spi_ports, PositionFeedbackConfig &config);

//reading fx
//non-volatile regs
int readZeroPosition(SPIPorts &spi_ports, int ifm_usec);
int readNumberPolePairs(SPIPorts &spi_ports, int ifm_usec);
int readSettings1(SPIPorts &spi_ports, int ifm_usec);
int readSettings2(SPIPorts &spi_ports, int ifm_usec);
int readRedundancyReg(SPIPorts &spi_ports, int ifm_usec);

//volatile regs
int readProgrammingReg(SPIPorts &spi_ports, int ifm_usec);
int readCORDICMagnitude(SPIPorts &spi_ports, int ifm_usec);
int readRotaryDiagnosticAndAutoGainControl(SPIPorts &spi_ports, int ifm_usec);
int readRotarySensorError(SPIPorts &spi_ports, int ifm_usec);
int readRotarySensorAngleWithoutCompensation(SPIPorts &spi_ports, int ifm_usec);
int readRotarySensorAngleWithCompensation(SPIPorts &spi_ports, int ifm_usec);

//writing fx
int writeSettings(SPIPorts &spi_ports, int ifm_usec, unsigned short address, unsigned short data);
int writeZeroPosition(SPIPorts &spi_ports, int ifm_usec, unsigned short data);
int writeNumberPolePairs(SPIPorts &spi_ports, int ifm_usec, unsigned short data);

#endif
