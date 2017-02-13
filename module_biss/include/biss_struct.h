/**
 * @file biss_struct.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ERROR                      0
#define SUCCESS                    1

#define SET_ALL_AS_QEI                 0b0011
#define SET_PORT1_AS_HALL_PORT2_AS_QEI 0b0010
#define SET_PORT1_AS_QEI_PORT2_AS_HALL 0b0001

#define BISS_POLARITY_NORMAL       1
#define BISS_POLARITY_INVERTED     -1

#define BISS_FRAME_BYTES           (( (3 + 2 + BISS_MULTITURN_LENGTH + BISS_SINGLETURN_LENGTH + BISS_STATUS_LENGTH + 6) -1)/32 + 1) //at least 3 bits + ack and start bits + data + crc
#define BISS_DATA_PORT_BIT         0            // bit number (0 = rightmost bit) when inputing from a multibit port
#define BISS_CLK_PORT_HIGH         (0b1000 | SET_PORT1_AS_HALL_PORT2_AS_QEI)    // high clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports
#define BISS_CLK_PORT_LOW          SET_PORT1_AS_HALL_PORT2_AS_QEI               // low  clock value when outputing the clock to a multibit port, with mode selection of ifm qei encoder and hall ports

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

typedef enum {
    BISS_DATA_PORT_1=0,
    BISS_DATA_PORT_2
} BISSDataPortConfig;

/**
 * @brief Structure type to define the BiSS Service configuration.
 */
typedef struct {
    int multiturn_length;       /**< Number of bits used for multiturn data */
    int multiturn_resolution;   /**< Number of bits of multiturn resolution */
    int singleturn_length;      /**< Number of bits used for singleturn data */
    int singleturn_resolution;  /**< Number of bits of singleturn resolution */
    int status_length;          /**< Number of bits used for status data */
    int crc_poly;               /**< CRC polynom in reverse representation:  x^0 + x^1 + x^4 is 0b1100 */
    int clock_dividend;         /**< BiSS output clock frequency dividend */
    int clock_divisor;          /**< BiSS output clock frequency divisor */
    int timeout;                /**< Timeout after a BiSS read in clock ticks */
    int velocity_loop;          /**< Velocity loop time in microseconds */
    int max_ticks;              /**< The count is reset to 0 if greater than this */
    BISSClockPortConfig clock_port_config; /**< Config of the biss clock port (4 or 1 bit) */
    BISSDataPortConfig  data_port_config;  /**< Config of the biss data port (4 or 1 bit) */
} BISSConfig;
