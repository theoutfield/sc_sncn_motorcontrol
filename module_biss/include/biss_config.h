/**
 * @file biss_config.h
 * @brief biss encoder Config Definitions
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#define ENC_CH1 1
#define ENC_CH2 2

#define BISS_DATA_PORT             ENC_CH2 //channel configuration, needed for the configuration of the clock output port
#define BISS_DATA_LENGTH           28
#define BISS_MULTITURN_LENGTH      13
#define BISS_MULTITURN_RESOLUTION  BISS_MULTITURN_LENGTH-1 //length - filling bits
#define BISS_SINGLETURN_LENGTH     13
#define BISS_SINGLETURN_RESOLUTION BISS_SINGLETURN_LENGTH
#define BISS_STATUS_LENGTH         2
#define BISS_CRC_POLY              0b110000

/**
 * @brief Interface definition for biss server
 */
interface i_biss {
    { int, int, unsigned int } get_position();
};

/**
 * @brief Structure definition for biss encoder
 */
typedef struct {
    int data_length;
    int multiturn_length;
    int multiturn_resolution;
    int singleturn_length;
    int singleturn_resolution;
    int status_length;
    int crc_poly;
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
