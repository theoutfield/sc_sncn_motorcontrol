/*
 * rem_16mt_config.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <rem_16mt_struct.h>

#define DEFAULT_SPI_CLOCK_DIV    32        // 250/32 / 2 = 4MHz

#define REM_16MT_USE_TIMESTAMP
#ifdef REM_16MT_USE_TIMESTAMP
#define REM_16MT_TIMEOUT           10   //micro seconds
#else
#define REM_16MT_TIMEOUT           38
#endif
