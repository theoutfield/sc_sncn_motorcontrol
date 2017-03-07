/*
 * rem_16mt_config.h
 *
 *  Created on: Sep 5, 2014
 *      Author: atena
 */

#pragma once
#include <rem_16mt_struct.h>

#define DEFAULT_SPI_CLOCK_DIV    32 /**< divisor for SPI clock frequency, (250/DIV)/2 MHz */
#define SPI_MASTER_MODE 1           /**< clock active high */
#define SPI_MASTER_SD_CARD_COMPAT 1 /**< MOSI high during input */
#define REM_16MT_MAX_RETRY        3 /**< Max number of read retry when checksum error */

#define REM_16MT_TIMEOUT           10   /**< Time to wait after read in micro seconds */
