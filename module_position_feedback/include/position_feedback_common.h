/**
 * @file position_feedback_common.h
 * @author Synapticon GmbH <support@synapticon.com>
 */


#pragma once


/**
 * @brief Type for Encoder Port
 *
 * There are two identical encoder ports which can be used for Hall, QEI and BiSS.
 */
typedef enum {
    ENCODER_PORT_1 = 0,  /**< Encoder port 1 (value should be 0) */
    ENCODER_PORT_2 = 1   /**< Encoder port 0 (value should be 1) */
} EncoderPortNumber;




/**
 * @brief Type for Sensor Error
 *
 */
typedef enum {
    SENSOR_NO_ERROR=0,
    SENSOR_REM_16MT_WEAK_MAG_FIELD_ERROR=0x1,
    SENSOR_REM_16MT_MT_COUNTER_ERROR=0x2,
    SENSOR_REM_16MT_ST_CORDIC_ERROR=0x3,
    SENSOR_REM_16MT_MT_SPEED_OVERLOW_ERROR=0x4,
    SENSOR_REM_16MT_FILTER_CONFIG_ERROR=0xA,
    SENSOR_REM_16MT_FILTER_SPEED_OVERLOW_ERROR=0xB,
    SENSOR_REM_16MT_UNKNOWN_CMD_ERROR=0xD,
    SENSOR_REM_16MT_CONFIG_ERROR=0xE,
    SENSOR_BISS_ERROR_BIT_ERROR,
    SENSOR_BISS_WARNING_BIT_ERROR,
    SENSOR_BISS_ERROR_AND_WARNING_BIT_ERROR,
    SENSOR_BISS_NO_ACK_BIT_ERROR,
    SENSOR_BISS_NO_START_BIT_ERROR,
    SENSOR_CHECKSUM_ERROR
} SensorError;
