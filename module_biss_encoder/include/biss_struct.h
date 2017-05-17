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
 * @brief Type for the BiSS clock port config
 *
 * There are 6 possible BiSS clock port configurations.
 * The first four use the GPIO ports 0 to 3. The value is used as the index of the gpio port array (for example gpio_ports[BISS_CLOCK_PORT_EXT_D2])
 * The last two are the third and fourth bits of the hall_enc_select_port. The value is used as a mask for outputing to this port (for example hall_enc_select_port <: BISS_CLOCK_PORT_EXT_D4)
 * This means that the values of this enum are significant and should not be changed!
 */
typedef enum {
    BISS_CLOCK_PORT_EXT_D0=0,       /**< GPIO port number 0 */
    BISS_CLOCK_PORT_EXT_D1=1,       /**< GPIO port number 1 */
    BISS_CLOCK_PORT_EXT_D2=2,       /**< GPIO port number 2 */
    BISS_CLOCK_PORT_EXT_D3=3,       /**< GPIO port number 3 */
    BISS_CLOCK_PORT_EXT_D4=0b0100,  /**< hall_enc_select_port 3rd bit */
    BISS_CLOCK_PORT_EXT_D5=0b1000   /**< hall_enc_select_port 4th bit */
} BISSClockPortConfig;

/**
 * @brief Structure type to define the BiSS sensor configuration.
 */
typedef struct {
    int multiturn_resolution;   /**< Number of bits of multiturn resolution */
    int singleturn_resolution;  /**< Number of bits of singleturn resolution */
    int filling_bits;           /**< Number of filling bits between the singleturn data status data*/
    int crc_poly;               /**< CRC polynom in reverse representation:  x^0 + x^1 + x^6 is 0b110000 */
    int clock_frequency;        /**< BiSS output clock frequency in kHz, supported frequencies depend on IFM Tile frequency */
    int timeout;                /**< Timeout after a BiSS read in microseconds */
    int busy;                   /**< maximum number of bits to read before the start bit (= maximum duration of ACK bit) */
    BISSClockPortConfig   clock_port_config;        /**< Configure of the biss clock port (GPIO or hall_enc_select_port) */
    EncoderPortNumber     data_port_number;         /**< Configure which port is used for the biss input data */
    EncoderPortSignalType data_port_signal_type;    /**< Configure of the data port signal type (RS422 (differential) or TTL) */
} BISSConfig;
