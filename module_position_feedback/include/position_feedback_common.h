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
