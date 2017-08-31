/**
 * @file qei_service.h
 * @brief Incremental Encoder Service Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <position_feedback_common.h>

/**
* @brief Definition for referring to the Encoder sensor.
*/

#define QEI_CHANGES_PER_TICK     4 //Quadrature encoder

#define QEI_ERROR                    0
#define QEI_SUCCESS                  1

#define FILTER_LENGTH_QEI        8
#define FILTER_LENGTH_QEI_PWM    8

#define QEI_RPM_CONST            1000*60
#define QEI_PWM_RPM_CONST        18000*60

#define QEI_RAW_POS_REQ          1
#define QEI_ABSOLUTE_POS_REQ     2
#define QEI_VELOCITY_REQ         3
#define QEI_VELOCITY_PWM_RES_REQ 4
#define SYNC                     5
#define SET_OFFSET               6
#define QEI_RESET_COUNT          7


/**
 * @brief Type for the sort of Encoder index.
 *
 * 2 is for two signals A and B
 * 3 is for three signals A, B and I for index
 */
typedef enum {
    QEI_WITH_NO_INDEX = 2,  /**< Encoder with no index signal. */
    QEI_WITH_INDEX    = 3   /**< Encoder with index signal. */
} QEI_IndexType;


/**
 * @brief Structure type to define the Encoder configuration.
 */
typedef struct {
    QEI_IndexType   number_of_channels; /**< Encoder index type. */
    int ticks_lost_threshold;           /**< Number of ticks that is allowed to be lost */
    EncoderPortNumber port_number;      /**< Configure which input port is used */
    EncoderPortSignalType signal_type;  /**< Configure of the input port signal type (RS422 (differential) or TTL) */
} QEIConfig;
