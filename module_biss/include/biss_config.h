/**
 * @file biss_config.h
 * @brief biss encoder Config Definitions
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>

#define ENC_CH1 1
#define ENC_CH2 2                 //FIXME use a normal 1-bit output port
#define BISS_DATA_PORT             ENC_CH2 //channel configuration, needed for the configuration of the clock output port


#define BISS_MULTITURN_RESOLUTION  12
#define BISS_SINGLETURN_RESOLUTION 13
#define BISS_STATUS_LENGTH         2
#define BISS_MULTITURN_LENGTH      BISS_MULTITURN_RESOLUTION+1 //resolution + filling bits
#define BISS_SINGLETURN_LENGTH     BISS_SINGLETURN_RESOLUTION
#define BISS_CRC_POLY              0b110000
#define BISS_POLARITY              BISS_POLARITY_NORMAL
#define BISS_MAX_TICKS             (1 << (BISS_MULTITURN_LENGTH -1 + BISS_SINGLETURN_LENGTH))
#define BISS_CLOCK_DIVIDEND        250          // BiSS output clock frequercy: dividend/divisor in MHz
#define BISS_CLOCK_DIVISOR         128          // supported frequencies are (tile frequency) / 2^n
#define BISS_TIMEOUT               12*USEC_FAST // BiSS timeout in clock ticks
#define BISS_VELOCITY_LOOP         1000         // velocity loop time in microseconds
#define BISS_OFFSET_ELECTRICAL     2670         // offset to align with the electrical 0 of the motor, range [ 0 - 4095 ]


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
 * @brief struct definition for velocity calculation from biss sensor
 */
typedef struct BISS_VELOCITY_PARAM
{
    int previous_position;
    int old_difference;
    int filter_buffer[8];
    int index;
    int filter_length;
} biss_velocity_par;

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
