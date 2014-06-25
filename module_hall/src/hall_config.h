/**
 * @file hall_config.h
 * @brief Hall Sensor Config Definitions
 * @author Ludwig Orgler <lorgler@synapticon.com>
 * @author Pavan Kanajar <pkanajar@synapticon.com>
 */

#pragma once

#define RPM_CONST 						 60000000  		// 60s / 1us
#define FILTER_LENGTH_HALL 						16
#define RESET_HALL_COUNT						9

typedef enum {
    HALL_POS_REQ,
    HALL_VELOCITY_REQ,
    HALL_ABSOLUTE_POS_REQ,
    HALL_FILTER_PARAM_REQ,
} hall_command_t;

 /**
 * @brief Structure definition for hall sensor
 */
typedef struct {
    int pole_pairs;
    int max_ticks_per_turn;
    int max_ticks;
    int sensor_polarity;
} hall_par;
