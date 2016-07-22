/**
 * @file hall_struct.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

/**
* @brief Definition for referring to the Hall sensor.
*/
#define HALL_SENSOR             1

#define ERROR                   0
#define SUCCESS                 1

#define RPM_CONST           60000000 // 60s / 1us
#define FILTER_LENGTH_HALL  16
#define PULL_PERIOD_USEC 12


#define HALL_TICKS_PER_ELECTRICAL_ROTATION 4096

/**
 * Structure type for Hall Service configuration
 */
typedef struct {
    int pole_pairs; /**< Number of pole pairs in your motor. */
    int enable_push_service;
} HallConfig;
