/**
 * @file qei_service.h
 * @brief Incremental Encoder Service Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <user_config.h>

/**
* @brief Definition for referring to the Encoder sensor.
*/

#define QEI_CHANGES_PER_TICK     4 //Quadrature encoder

#define ERROR                    0
#define SUCCESS                  1

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

#define QEI_POLARITY_NORMAL      1
#define QEI_POLARITY_INVERTED   -1

#define QEI_PORT_AS_TTL           0b0000
#define QEI_PORT_AS_RS422         0b0010

#define QEI_USEC                 IFM_TILE_USEC


/**
 * @brief Type for the kind of Encoder output signals.
 */
typedef enum {
    QEI_RS422_SIGNAL = 11,  /**< Encoder signal output over RS422 (differential). */
    QEI_TTL_SIGNAL = 22     /**< Encoder signal output over standard TTL signal.  */
} QEI_SignalType;

/**
 * @brief Type for the sort of Encoder index.
 */
typedef enum {
    QEI_WITH_NO_INDEX = 3,  /**< Encoder with no index signal. */
    QEI_WITH_INDEX  = 4     /**< Encoder with index signal.  */
} QEI_IndexType;

/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    int ticks_resolution;       /**< Encoder resolution [pulses/revolution]. */
    QEI_IndexType index_type;   /**< Encoder index type. */
    int sensor_polarity;        /**< Encoder direction. */
    QEI_SignalType signal_type; /**< Encoder output signal type (if applicable in your SOMANET device). */
    int enable_push_service;
} QEIConfig;
