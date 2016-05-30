/*
 * rotary_sensor.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once

#include <stdint.h>
#include <refclk.h>

//encoder config
#define CONTELEC_OFFSET          0//3696
#define CONTELEC_POLARITY        CONTELEC_POLARITY_NORMAL//CONTELEC_POLARITY_NORMAL
#define CONTELEC_USEC            USEC_FAST
#define CONTELEC_TIMEOUT         40*CONTELEC_USEC
#define CONTELEC_RESOLUTION      16
#define CONTELEC_VELOCITY_LOOP   1000
#define CONTELEC_FILTER          0x05

//SPI config
#define DEFAULT_SPI_CLOCK_DIV 32        // 250/DIV MHz
#define SPI_MASTER_MODE 1 //clock active high
#define SPI_MASTER_SD_CARD_COMPAT 1 //MOSI high during input

#define ERROR       0
#define SUCCESS     1

#define CONTELEC_POLARITY_NORMAL      0
#define CONTELEC_POLARITY_INVERTED    1



#ifdef __XC__

#include <spi_master.h>
#include <memory_manager.h>
#include <position_feedback_service.h>


int contelec_encoder_init(SPIPorts &contelec_ports, CONTELECConfig config);
void init_spi_ports(SPIPorts &spi_ports);


[[combinable]]
void contelec_service(SPIPorts &contelec_ports, CONTELECConfig &config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3]);

{ char, int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &contelec_ports);

void contelec_encoder_write(SPIPorts &contelec_ports, int opcode, int data, int data_bits);

#endif
