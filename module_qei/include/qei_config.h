/**
 * @file qei_config.h
 * @brief QEI Sensor Config Definitions
 * @author Pavan Kanajar <pkanajar@synapticon.com>
 * @author Martin Schwarz <mschwarz@synapticon.com>
 */

#pragma once

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
 * @brief struct definition for quadrature sensor
 */
typedef struct {
    int max_ticks_per_turn;
    int real_counts;
    int max_ticks;      // paramater allows for more turns
    int index;          // no_index - 0 index - 1
    int poles;
    int sensor_polarity;
} qei_par;

/**
 * @brief struct definition for velocity calculation from qei sensor
 */
typedef struct QEI_VELOCITY_PARAM
{
    int previous_position;
    int old_difference;
    int filter_buffer[8];
    int index;
    int filter_length;
} qei_velocity_par;
