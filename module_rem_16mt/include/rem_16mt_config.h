/*
 * rem_16mt_config.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <rem_16mt_struct.h>

#define REM_16MT_USE_TIMESTAMP

#define REM_16MT_OFFSET          0
#define REM_16MT_POLARITY        REM_16MT_POLARITY_NORMAL//REM_16MT_POLARITY_INVERTED
#define REM_16MT_RESOLUTION      16
#ifdef REM_16MT_USE_TIMESTAMP
#define REM_16MT_TIMEOUT         10*REM_16MT_USEC
#define REM_16MT_VELOCITY_LOOP   53
#else
#define REM_16MT_TIMEOUT         38*REM_16MT_USEC
#define REM_16MT_VELOCITY_LOOP   65
#endif
#define REM_16MT_FILTER          0x02
#define DEFAULT_SPI_CLOCK_DIV    32        // 250/32 / 2 = 4MHz
