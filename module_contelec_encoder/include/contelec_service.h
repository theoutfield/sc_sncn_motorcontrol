/*
 * contelec_service.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

int contelec_encoder_init(SPIPorts &spi_ports, CONTELECConfig config);
void init_spi_ports(SPIPorts &spi_ports);


//[[combinable]]
void contelec_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3]);

#ifdef CONTELEC_USE_TIMESTAMP
{ char, int, unsigned int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &spi_ports);
#else
{ char, int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &spi_ports);
#endif

void contelec_encoder_write(SPIPorts &spi_ports, int opcode, int data, int data_bits);

#endif
