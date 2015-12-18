/**
 * @file biss_config.h
 * @brief biss encoder Config Definitions
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>

#define SET_ALL_AS_QEI 0b0011
#define SET_PORT1_AS_HALL_PORT2_AS_QEI 0b0010
#define SET_PORT1_AS_QEI_PORT2_AS_HALL 0b0001


#define BISS_MULTITURN_RESOLUTION  0
#define BISS_SINGLETURN_RESOLUTION 18
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION + 1
#define BISS_FRAME_BYTES           (( (3 + 2 + BISS_MULTITURN_LENGTH + BISS_SINGLETURN_LENGTH + BISS_STATUS_LENGTH + 6) -1)/32 + 1) //at least 3 bits + ack and start bits + data + crc
#define BISS_POLARITY              BISS_POLARITY_INVERTED
#define BISS_MAX_TICKS             0x7fffffff   // the count is reset to 0 if greater than this
#define BISS_CRC_POLY              0b110000     // poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
#define BISS_DATA_PORT_BIT         1            // bit number (0 = rightmost bit) when inputing from a multibit port
#define BISS_CLK_PORT_MASK         (0b0100 | SET_PORT1_AS_QEI_PORT2_AS_HALL) // bit mask when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequercy: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         128          // supported frequencies are (tile frequency) / 2^n
#define BISS_USEC                  USEC_FAST    // number of ticks in a microsecond
#define BISS_VELOCITY_LOOP         1000         // velocity loop time in microseconds
#define BISS_TIMEOUT               20*BISS_USEC // BiSS timeout in clock ticks
#define BISS_OFFSET_ELECTRICAL     1800


/**
 * @brief Structure definition for biss encoder
 */
typedef struct {
    int multiturn_length;
    int multiturn_resolution;
    int singleturn_length;
    int singleturn_resolution;
    int status_length;
    int crc_poly;
    int poles;
    int polarity;
    int clock_dividend;
    int clock_divisor;
    int timeout;
    int velocity_loop;
    int max_ticks;
    int offset_electrical;
} biss_par;


/**
 * enum for the several status informations
 * of the read_biss_sensor_data() function
 */
enum {
  NoError,
  CRCError,
  NoAck,
  NoStartBit,
};

enum { BISS_POLARITY_NORMAL, BISS_POLARITY_INVERTED }; /* Encoder polarity */
