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
int readZeroPosition(SPIPorts &spi_ports);
int readNumberPolePairs(SPIPorts &spi_ports);
int readSettings1(SPIPorts &spi_ports);
int readSettings2(SPIPorts &spi_ports);
int readRedundancyReg(SPIPorts &spi_ports);

//volatile regs
int readProgrammingReg(SPIPorts &spi_ports);
int readCORDICMagnitude(SPIPorts &spi_ports);
int readRotaryDiagnosticAndAutoGainControl(SPIPorts &spi_ports);
int readRotarySensorError(SPIPorts &spi_ports);
int readRotarySensorAngleWithoutCompensation(SPIPorts &spi_ports);
int readRotarySensorAngleWithCompensation(SPIPorts &spi_ports);

//writing fx
int writeSettings1(SPIPorts &spi_ports, unsigned short data);
int writeSettings2(SPIPorts &spi_ports, unsigned short data);
int writeZeroPosition(SPIPorts &spi_ports, unsigned short data);
int writeNumberPolePairs(SPIPorts &spi_ports, unsigned short data);

void rem_14_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3]);

#endif
