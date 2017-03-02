/**
 * @file biss_config.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <biss_struct.h>

/**
 * @brief Number of 32 bit bytes needed to save the BiSS frame before processing
 *
 * It must be large enough to hold: CDS bit + multiturn and singleturn data with filling bits + error and warning bits and the crc (6)
 * 2 bytes should be enough for a sensor up to 55 bits of position data (1 + 55 + 2 + 6 = 64)
 *
 */
#define BISS_FRAME_BYTES    2

#define BISS_DATA_PORT_BIT  0    /**< Bit number (0 = rightmost bit) when inputing from a multibit port */
#define BISS_STATUS_BITS    2    /**< Number of bits used for status data (usually 2)*/
