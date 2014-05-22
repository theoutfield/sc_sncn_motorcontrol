
/**
 * \file bldc_motor_config.h
 * \brief Motor Control config file (define your the motor specifications here)
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
 

#ifndef __DC_MOTOR_CONFIG__H__ADC_TEST
#define __DC_MOTOR_CONFIG__H__ADC_TEST
#include <print.h>
#include <internal_config.h>

#pragma once

/**
 * Define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS  				8		// Number of pole pairs
#define MAX_NOMINAL_SPEED  			5260	// rpm
#define MAX_NOMINAL_CURRENT  		2		// A
#define MOTOR_TORQUE_CONSTANT 		34    	// mNm/A

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO  				26		// if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION 			4000	// 4 x Max count of Incremental Encoder (4X decoding - quadrature mode)

/* Define your Incremental Encoder type (QEI_WITH_INDEX/ QEI_WITH_NO_INDEX) */
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY			NORMAL

/* Somanet IFM Internal Config:  Specifies the current sensor resolution per Ampere
 *  (DC300_RESOLUTION / DC100_RESOLUTION / OLD_DC300_RESOLUTION) */
#define IFM_RESOLUTION				DC100_RESOLUTION

/* Commutation offset (range 0-4095) (HALL sensor based commutation) */
#define COMMUTATION_OFFSET_CLK		683
#define COMMUTATION_OFFSET_CCLK		2731

/* Motor Winding type (STAR_WINDING/DELTA_WINDING) */
#define WINDING_TYPE				DELTA_WINDING


#define MAX_POSITION_LIMIT 			POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO		// ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT 			-POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO		// ticks (min range: -2^30, limited for safe operation)

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

/**
 * \brief initialize QEI sensor
 *
 * \param qei_params struct defines the resolution for quadrature encoder (QEI),
 * 			gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par &qei_params);



#endif
