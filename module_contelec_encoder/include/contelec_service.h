/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#include <contelec_config.h>


#ifdef __XC__

#include <spi_master.h>
#include <memory_manager.h>

typedef struct
{
    spi_master_interface spi_interface;
    out port ?slave_select;
} SPIPorts;

#include <position_feedback_service.h>

int contelec_encoder_init(PositionFeedbackPorts &position_feedback_ports, CONTELECConfig config);
void init_spi_ports(PositionFeedbackPorts &position_feedback_ports);


[[combinable]]
void contelec_service(PositionFeedbackPorts &position_feedback_ports, CONTELECConfig &config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3]);

{ char, int, unsigned int, unsigned int } contelec_encoder_read(PositionFeedbackPorts &position_feedback_ports);

void contelec_encoder_write(PositionFeedbackPorts &position_feedback_ports, int opcode, int data, int data_bits);

#endif
