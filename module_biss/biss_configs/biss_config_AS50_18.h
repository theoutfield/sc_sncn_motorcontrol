/**
 * @file biss_config.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once
#include <biss_struct.h>

#define BISS_MULTITURN_RESOLUTION  0
#define BISS_SINGLETURN_RESOLUTION 18
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION + 1
#define BISS_POLARITY              BISS_POLARITY_NORMAL
#define BISS_MAX_TICKS             0x7fffffff   // the count is reset to 0 if greater than this
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequency: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         26           // supported frequencies are (tile frequency) / 2n
#define BISS_VELOCITY_LOOP         100         // velocity loop time in microseconds
#define BISS_TIMEOUT               20*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     0
