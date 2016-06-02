/*
 * contelec_config.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <contelec_struct.h>

#define CONTELEC_OFFSET          0
#define CONTELEC_POLARITY        CONTELEC_POLARITY_NORMAL
#define CONTELEC_TIMEOUT         38*CONTELEC_USEC
#define CONTELEC_RESOLUTION      16
#define CONTELEC_VELOCITY_LOOP   100
#define CONTELEC_FILTER          0x05
#define DEFAULT_SPI_CLOCK_DIV    32        // 250/32 / 2 = 4MHz
