
/**
 *
 * \file bldc_motor_config.h
 *	Motor Control config file
 *
 *	Please define your the motor specifications here
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
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

#ifndef __DC_MOTOR_CONFIG__H__SM
#define __DC_MOTOR_CONFIG__H__SM
#include <print.h>
#include <internal_config.h>

#pragma once

/*
 * define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS  				8
#define MAX_NOMINAL_SPEED  			5260				// rpm
#define MAX_NOMINAL_CURRENT  		2					// A
#define MOTOR_TORQUE_CONSTANT 		38    				// mNm/A

/* If you have any gears added specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (optional)
 */
#define GEAR_RATIO  				18					// if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION 			4000				// 4 x Max count of Quadrature Encoder (4X decoding)

/* Choose Position/Velocity Sensor */
#define SENSOR_USED 				HALL 				// QEI

/*Define your Encoder type*/
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX		// QEI_WITH_NO_INDEX


/* Somanet IFM Internal Config */
#define IFM_RESOLUTION				DC100_RESOLUTION 	 // DC300_RESOLUTION   /* Specifies the current sensor resolution/A */


/*Changes direction of the motor drive*/
#define POLARITY 					1					// 1 / -1

/*Commutation offset (range 0-4095) */
#define COMMUTATION_OFFSET_CLK		683
#define COMMUTATION_OFFSET_CCLK		2731

/*Motor Winding type*/
#define WINDING_TYPE				DELTA_WINDING		// 1: star-type(Y) or 2: delta-type STAR_WINDING//

/* Profile defines (optional) */
#define MAX_PROFILE_VELOCITY  		MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY			1000				// rpm
#define MAX_ACCELERATION   			4000   				// rpm/s
#define PROFILE_ACCELERATION		2000				// rpm/s
#define PROFILE_DECELERATION  		2000				// rpm/s
#define QUICK_STOP_DECELERATION 	2000				// rpm/s
#define PROFILE_TORQUE_SLOPE		66					// (desired torque_slope/torque_constant)  * IFM resolution


/* Control specific constants/variables */
	/*Torque Control (Mandatory if Torque control used)*/
#define TORQUE_Kp_NUMERATOR 	   	20
#define TORQUE_Kp_DENOMINATOR  		10
#define TORQUE_Ki_NUMERATOR    		11
#define TORQUE_Ki_DENOMINATOR  		110
#define TORQUE_Kd_NUMERATOR    		1
#define TORQUE_Kd_DENOMINATOR  		10

	/*Velocity Control (Mandatory if Velocity control used)*/
#define VELOCITY_Kp_NUMERATOR 	 	5
#define VELOCITY_Kp_DENOMINATOR  	10
#define VELOCITY_Ki_NUMERATOR    	5
#define VELOCITY_Ki_DENOMINATOR  	100
#define VELOCITY_Kd_NUMERATOR    	0
#define VELOCITY_Kd_DENOMINATOR  	1

#define VELOCITY_FILTER_SIZE        8  	//default (could be changed upto 16)

	/*Position Control (Mandatory if Position control used)*/
#define POSITION_Kp_NUMERATOR 	 	180
#define POSITION_Kp_DENOMINATOR  	2000
#define POSITION_Ki_NUMERATOR    	50
#define POSITION_Ki_DENOMINATOR  	102000
#define POSITION_Kd_NUMERATOR    	100
#define POSITION_Kd_DENOMINATOR  	10000

#define MAX_POSITION_LIMIT 			350		// degree should not exceed 359
#define MIN_POSITION_LIMIT 			-350	// degree should not exceed -359




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
