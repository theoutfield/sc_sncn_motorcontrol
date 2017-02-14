/*
 * rem_16mt_service.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

int rem_16mt_init(SPIPorts &spi_ports, PositionFeedbackConfig &config);
void init_spi_ports(SPIPorts &spi_ports);

#ifdef REM_16MT_USE_TIMESTAMP
{ char, int, unsigned int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports, int ifm_usec);
#else
{ char, int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports, int ifm_usec);
#endif

void rem_16mt_write(SPIPorts &spi_ports, int opcode, int data, int data_bits, int ifm_usec);

#endif
