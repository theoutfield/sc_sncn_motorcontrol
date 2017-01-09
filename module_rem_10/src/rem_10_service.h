#pragma once

#include <stdint.h>
#include "refclk.h"
#include <position_feedback_service.h>

void initspiPorts(SPIPorts &spi_ports);
uint8_t calcParity(unsigned short bitStream);
unsigned short add_EvenParity(unsigned short bitStream);
unsigned char check_EvenParity(unsigned short bitStream);
short ReadTransactionSPI(SPIPorts &spi_ports, unsigned short reg);
short WriteTransactionSPI(SPIPorts &spi_ports, unsigned short reg, unsigned short data);
int checkWOWBit(SPIPorts &spi_ports);
int readAngleValue(SPIPorts &spi_ports);
int readAutomaticGainControl(SPIPorts &spi_ports);
int porCellDeactivate(SPIPorts &spi_ports);
int initas5050a(SPIPorts &spi_ports);

void as5050a_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3]);
