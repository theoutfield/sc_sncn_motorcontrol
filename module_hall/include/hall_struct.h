/**
 * @file hall_struct.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <position_feedback_common.h>

#define ERROR                   0
#define SUCCESS                 1

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16

#define HALL_TICKS_PER_ELECTRICAL_ROTATION 4096

// Hall_states
#define HALL_STATE_0  4
#define HALL_STATE_1  6
#define HALL_STATE_2  2
#define HALL_STATE_3  3
#define HALL_STATE_4  1
#define HALL_STATE_5  5

// defined angle for each hall state
#define HALL_ANGLE_0 3755
#define HALL_ANGLE_1 341
#define HALL_ANGLE_2 1024
#define HALL_ANGLE_3 1707
#define HALL_ANGLE_4 2389
#define HALL_ANGLE_5 3072

// longest (acceptable) electrical period
#define HALL_PERIOD_MAX   1000000
#define HALL_TRANSITION_PERIOD_MAX HALL_PERIOD_MAX/6


/**
 * @brief Structure type to define the Encoder Service configuration.
 */
typedef struct {
    EncoderPortType port_config;    /**< Config which input port is used */
} HallConfig;


// variables related to hall measurement
typedef struct
{

    int hall_sector;
    int hall_period;

    int hall_transition_period;
    int hall_last_transition_period;

    int hall_pin_state;

    int hall_next_state;
    int hall_previous_state;

    int hall_direction_of_rotation;

    int hall_position;

    int hall_angle;
    int hall_increment;
    int hall_interpolated_angle;

    int hall_speed;
    int hall_speed_before_stopping;
    int hall_filtered_speed;

    unsigned int hall_transition_period_at_1rpm;

    unsigned int hall_f_clock;
    int hall_pole_pairs;
    int sensor_polarity;

    // filter constants
    int hall_filter_order;
    int hall_filter_index_newest;
    int h[3];
    int hall_filter_buffer[3];

}hall_variables;

