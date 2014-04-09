
/**
 * \file bldc_motor_config.h
 * \brief Motor Control config file (define your the motor specifications here)
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
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

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_NO_INDEX) */
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY			INVERTED

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
