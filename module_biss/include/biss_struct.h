/**
 * @file biss_struct.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <position_feedback_common.h>

#define SET_ALL_AS_QEI                 0b0011
#define SET_PORT1_AS_HALL_PORT2_AS_QEI 0b0010
#define SET_PORT1_AS_QEI_PORT2_AS_HALL 0b0001

#define BISS_CDS_BIT        1    /**< CDS bit length */

/**
 * @brief Type for the return status when reading BiSS data
 */
typedef enum {
    NoError=0,       /**< no error */
    CRCCorrected=1,  /**< CRC corrected  */
    CRCError=2,      /**< CRC mismatch  */
    NoAck,           /**< Ack bit not found. */
    NoStartBit       /**< Start bit not found */
} BISS_ErrorType;

typedef enum {
    BISS_CLOCK_PORT_EXT_D2=2,
    BISS_CLOCK_PORT_EXT_D3=3,
    BISS_CLOCK_PORT_EXT_D4=0b0100, //4
    BISS_CLOCK_PORT_EXT_D5=0b1000  //8
} BISSClockPortConfig;

/**
 * @brief Structure type to define the BiSS Service configuration.
 */
typedef struct {
    int multiturn_resolution;   /**< Number of bits of multiturn resolution */
    int singleturn_resolution;  /**< Number of bits of singleturn resolution */
    int filling_bits;           /**< Number of filling bits between the singleturn data status data*/
    int crc_poly;               /**< CRC polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100 */
    int clock_frequency;        /**< BiSS output clock frequency in kHz, supported frequencies depend on IFM Tile frequency */
    int timeout;                /**< Timeout after a BiSS read in clock ticks */
    int busy;                   /**< maximum number of bits to read before the start bit (= maximum duration of ack bit) */
    BISSClockPortConfig clock_port_config;  /**< Config of the biss clock port (4 or 1 bit) */
    EncoderPortNumber  data_port_number;    /**< Config with port is used for the biss data */
} BISSConfig;
