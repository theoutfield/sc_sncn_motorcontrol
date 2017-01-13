/*
 * serial_encoder_service.h
 *
 *  Created on: 26.11.2016
 *      Author: synapticon
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>

void serial_encoder_service(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4], int hall_enc_select_config, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3]);

#endif
