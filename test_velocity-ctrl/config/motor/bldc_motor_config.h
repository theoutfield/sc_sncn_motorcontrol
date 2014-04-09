
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

#ifndef __DC_MOTOR_CONFIG__H__VELOCITY
#define __DC_MOTOR_CONFIG__H__VELOCITY
#include <print.h>
#include <internal_config.h>

#pragma once

/**
 * Define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS  				4				// Number of pole pairs
#define MAX_NOMINAL_SPEED  			3000			// rpm
#define MAX_NOMINAL_CURRENT  		2				// A
#define MOTOR_TORQUE_CONSTANT 		34    			// mNm/A

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO  				100				// if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION 			4000			// 4 x Max count of Incremental Encoder (4X decoding - quadrature mode)

/* Position/Velocity Sensor Types (select your sensor type here)
 * (HALL/ QEI) */
#define SENSOR_USED 				QEI

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_NO_INDEX) */
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (NORMAL/INVERTED) */
#define QEI_SENSOR_POLARITY			INVERTED

/* Somanet IFM Internal Config:  Specifies the current sensor resolution per Ampere
 *  (DC300_RESOLUTION / DC100_RESOLUTION / OLD_DC300_RESOLUTION) */
#define IFM_RESOLUTION				DC300_RESOLUTION

/* Commutation offset (range 0-4095) (HALL sensor based commutation) */
#define COMMUTATION_OFFSET_CLK		770		// com neg
#define COMMUTATION_OFFSET_CCLK		2601    //com pos

/* Motor Winding type (STAR_WINDING/DELTA_WINDING) */
#define WINDING_TYPE				DELTA_WINDING

/* Changes direction of the motor drive  (1 /-1) */
#define POLARITY 					1

#define MAX_POSITION_LIMIT 			16000				// ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT 			-16000				// ticks (min range: -2^30, limited for safe operation)

/* Profile defines (Mandatory for profile modes) */
#define MAX_PROFILE_VELOCITY  		MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY			1001				// rpm
#define MAX_ACCELERATION   			3000    			// rpm/s
#define PROFILE_ACCELERATION		2002				// rpm/s
#define PROFILE_DECELERATION  		2004				// rpm/s
#define QUICK_STOP_DECELERATION 	2005				// rpm/s
#define PROFILE_TORQUE_SLOPE		66					// (desired torque_slope/torque_constant)  * IFM resolution


/* Control specific constants/variables */
	/* Velocity Control (Mandatory if Velocity control used)
	 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
	 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define VELOCITY_Kp_NUMERATOR 	 	5
#define VELOCITY_Kp_DENOMINATOR  	10
#define VELOCITY_Ki_NUMERATOR    	5
#define VELOCITY_Ki_DENOMINATOR  	100
#define VELOCITY_Kd_NUMERATOR    	0
#define VELOCITY_Kd_DENOMINATOR  	1

#define VELOCITY_FILTER_SIZE        8  	//default (could be changed upto 128)

	/* Torque Control (Mandatory if Torque control used)
	 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
	 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define TORQUE_Kp_NUMERATOR 	   	50
#define TORQUE_Kp_DENOMINATOR  		10
#define TORQUE_Ki_NUMERATOR    		11
#define TORQUE_Ki_DENOMINATOR  		110
#define TORQUE_Kd_NUMERATOR    		1
#define TORQUE_Kd_DENOMINATOR  		10

	/* Position Control (Mandatory if Position control used)
	 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
	 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define POSITION_Kp_NUMERATOR 	 	180
#define POSITION_Kp_DENOMINATOR  	2000
#define POSITION_Ki_NUMERATOR    	50
#define POSITION_Ki_DENOMINATOR  	102000
#define POSITION_Kd_NUMERATOR    	100
#define POSITION_Kd_DENOMINATOR  	10000






/**
 * \brief initialize Velocity sensor filter
 *
 * \param sensor_filter_par struct defines the velocity filter params
 */
void init_sensor_filter_param(filter_par &sensor_filter_par) ;

/**
 * \brief initialize QEI sensor
 *
 * \param qei_params struct defines the resolution for quadrature encoder (QEI),
 * 			gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par &qei_params);

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

/**
 * \brief initialize cyclic synchronous velocity params
 *
 * \param csv_params struct defines cyclic synchronous velocity params
 */
void init_csv_param(csv_par &csv_params);

/**
 * \brief initialize cyclic synchronous position params
 *
 * \param csp_params struct defines cyclic synchronous position params
 */
void init_csp_param(csp_par &csp_params);

/**
 * \brief initialize cyclic synchronous torque params
 *
 * \param cst_params struct defines cyclic synchronous torque params
 */
void init_cst_param(cst_par &cst_params);

/**
 * \brief initialize profile position params
 *
 * \param pp_params struct defines profileposition params
 */
void init_pp_params(pp_par &pp_params);

/**
 * \brief initialize profile velocity params
 *
 * \param pv_params struct defines profile velocity params
 */
void init_pv_params(pv_par &pv_params);

/**
 * \brief initialize profile torque params
 *
 * \param pt_params struct defines profile torque params
 */
void init_pt_params(pt_par &pt_params);

/**
 * \brief initialize torque control PID params
 *
 * \param torque_ctrl_params struct defines torque control PID params
 */
void init_torque_control_param(ctrl_par &torque_ctrl_params);

/**
 * \brief initialize velocity control PID params
 *
 * \param velocity_ctrl_params struct defines velocity control PID params
 */
void init_velocity_control_param(ctrl_par &velocity_ctrl_params);

/**
 * \brief initialize position control PID params
 *
 * \param position_ctrl_params struct defines position control PID params
 */
void init_position_control_param(ctrl_par &position_ctrl_params);

#endif
