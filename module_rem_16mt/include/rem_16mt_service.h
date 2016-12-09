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


//[[combinable]]
void rem_16mt_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3]);

#ifdef REM_16MT_USE_TIMESTAMP
{ char, int, unsigned int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports);
#else
{ char, int, unsigned int, unsigned int } rem_16mt_read(SPIPorts &spi_ports);
#endif

void rem_16mt_write(SPIPorts &spi_ports, int opcode, int data, int data_bits);

#endif
