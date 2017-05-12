/*
 * serial_encoder_service.h
 *
 *  Created on: 26.11.2016
 *      Author: synapticon
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

/**
 * @brief Service to read and process data from an Position Sensor with a serial interface (SPI or BiSS).
 *
 * @param qei_hall_port_1 BiSS input port number 1
 * @param qei_hall_port_2 BiSS input port number 2
 * @param hall_enc_select_port port used to select the mode (differential or not) of Hall/QEI/BiSS ports
 * @param spi_ports SPI ports and clock blocks
 * @param gpio_ports GPIO ports array
 * @param hall_enc_select_config config to select the mode (differential or not) of Hall/QEI/BiSS ports
 * @param position_feedback_config Configuration for the service.
 * @param i_shared_memory Client interface to write the position data to the shared memory.
 * @param i_position_feedback Server interface used by clients for configuration and direct position read.
 * @param gpio_on Set to 1 to enable GPIO read/write.
 */
void serial_encoder_service(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4],
                int hall_enc_select_config, PositionFeedbackConfig &position_feedback_config,
                client interface shared_memory_interface ?i_shared_memory,
                interface PositionFeedbackInterface server i_position_feedback[3],
                int gpio_on);
#endif
