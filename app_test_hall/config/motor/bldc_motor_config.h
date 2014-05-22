
/**
 * \file bldc_motor_config.h
 * \brief Motor Control config file (Please define your the Hall Sensor specifications here)
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#ifndef __DC_MOTOR_CONFIG__H__HALL_TEST
#define __DC_MOTOR_CONFIG__H__HALL_TEST
#include <print.h>
#include <internal_config.h>

#pragma once


#define POLE_PAIRS  				4 		// Number of pole pairs

/**
 * If you have any gears added, specify gear-ratio
 */
#define GEAR_RATIO 					10		// if no gears are attached - set to gear ratio to 1

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_NO_INDEX) */
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY			NORMAL

#define MAX_POSITION_LIMIT 			POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO		// ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT 			-POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO		// ticks (min range: -2^30, limited for safe operation)


/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);


#endif
