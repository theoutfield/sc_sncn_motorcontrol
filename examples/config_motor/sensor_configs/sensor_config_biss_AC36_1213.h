// default sensor config

#pragma once

#define BISS_CONFIG

#define BISS_MULTITURN_RESOLUTION  12
#define BISS_SINGLETURN_RESOLUTION 13
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION + 1 //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequency: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         32           // supported frequencies are (tile frequency) / 2n
#define BISS_VELOCITY_COMPUTE_PERIOD 50         // velocity loop time in microseconds
#define BISS_TIMEOUT               16*IFM_TILE_USEC // BiSS timeout in clock ticks
#define BISS_CLOCK_PORT            BISS_CLOCK_PORT_EXT_D5
#define BISS_DATA_PORT             BISS_DATA_PORT_2
