/**
 * @file hall_config.h
 * @brief Hall Sensor Config Definitions
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Pavan Kanajar <pkanajar@synapticon.com>
 */

#pragma once

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16

/**
 * Client/server interaction commands/tokens
 */
enum {
    HALL_POS_REQ,         //!< Position request token
    HALL_ABSOLUTE_POS_REQ,//!< Position request token
    HALL_VELOCITY_REQ,    //!< Velocity request token
    HALL_RESET_COUNT_REQ,     //!< Reset hall server ticks count
    HALL_FILTER_PARAM_REQ,//!< Filter length request token
};

/**
 * @brief Structure definition for hall sensor
 */
typedef struct {
    int pole_pairs;
    int max_ticks_per_turn;
    int max_ticks;
    int sensor_polarity;
} hall_par;
