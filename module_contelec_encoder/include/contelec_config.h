/*
 * contelec_config.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <contelec_struct.h>

#define CONTELEC_OFFSET          0
#define CONTELEC_POLARITY        CONTELEC_POLARITY_NORMAL//CONTELEC_POLARITY_INVERTED
#define CONTELEC_TIMEOUT         10*CONTELEC_USEC
#define CONTELEC_RESOLUTION      16
#define CONTELEC_VELOCITY_LOOP   62
#define CONTELEC_FILTER          0x02
#define DEFAULT_SPI_CLOCK_DIV    32        // 250/32 / 2 = 4MHz
