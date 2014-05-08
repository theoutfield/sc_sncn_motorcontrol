
/**
 * \file bldc_motor_config.h
 * \brief Motor Control config file (Please define your the Incremental Encoder (QEI)
 *   specifications here)
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#ifndef __DC_MOTOR_CONFIG__H__QEI_TEST
#define __DC_MOTOR_CONFIG__H__QEI_TEST
#include <print.h>
#include <internal_config.h>

#pragma once

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO  				10		// if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION 			4000	// 4 x Max count of Quadrature Encoder (4X decoding)

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_NO_INDEX) */
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY			NORMAL

#define MAX_POSITION_LIMIT 			GEAR_RATIO*ENCODER_RESOLUTION		// ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT 			-GEAR_RATIO*ENCODER_RESOLUTION		// ticks (min range: -2^30, limited for safe operation)


/**
 * \brief initialize QEI sensor
 *
 * \param qei_params struct defines the resolution for quadrature encoder (QEI),
 * 			gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par &qei_params);


#endif
