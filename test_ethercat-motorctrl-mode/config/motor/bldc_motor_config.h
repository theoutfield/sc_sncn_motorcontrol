
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

#ifndef __DC_MOTOR_CONFIG__H__ETHERCAT_MOTOR_DRIVE
#define __DC_MOTOR_CONFIG__H__ETHERCAT_MOTOR_DRIVE
#include <print.h>
#include <internal_config.h>

#pragma once

/*
 * define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS  				8
#define MAX_NOMINAL_SPEED  			4000	// rpm
#define MAX_NOMINAL_CURRENT  		2		// A
#define MOTOR_TORQUE_CONSTANT 		34    	// mNm/A

/* If you have any gears added specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (optional)
 */
#define GEAR_RATIO  				26		// if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION 			4000	// 4 x Max count of Quadrature Encoder (4X decoding)

/* Choose Position/Velocity Sensor */
#define SENSOR_USED 				HALL 	// QEI

/*Define your Encoder type*/
#define QEI_SENSOR_TYPE  			QEI_WITH_INDEX	// QEI_WITH_NO_INDEX

/*Somanet IFM Internal Config*/
#define IFM_RESOLUTION				DC100_RESOLUTION  // DC300_RESOLUTION   /* Specifies the current sensor resolution/A

/*Changes direction of the motor drive*/
#define POLARITY 					1		// 1 / -1

/* Profile defines (optional) */
#define MAX_PROFILE_VELOCITY  		MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY 			1000	// rpm
#define MAX_ACCELERATION   			5000    // rpm/s
#define PROFILE_ACCELERATION		2000	// rpm/s
#define PROFILE_DECELERATION   		2000	// rpm/s
#define QUICK_STOP_DECELERATION 	2000	// rpm/s
#define PROFILE_TORQUE_SLOPE		200		// (desired torque_slope/torque_constant)  * IFM resolution
#define MAX_TORQUE					MOTOR_TORQUE_CONSTANT * IFM_RESOLUTION * MAX_NOMINAL_CURRENT

/* Control specific constants/variables */
	/*Torque Control (Mandatory if Torque control used)*/
#define TORQUE_KP					81920		//	(50/10)  * 16384
#define TORQUE_KI					1638		//	(11/110) * 16384
#define TORQUE_KD					1638		//	(1/10)   * 16384

	/*Velocity Control (Mandatory if Velocity control used)*/
#define VELOCITY_KP				 	8192       	//  (5/10)   * 16384
#define VELOCITY_KI    				819			//  (5/100)  * 16384
#define VELOCITY_KD				   	0			//	(0)      * 16384

	/*Position Control (Mandatory if Position control used)*/
#define POSITION_KP	 				1474		//	(180/2000)  * 16384
#define POSITION_KI   				8			//	(50/102000) * 16384
#define POSITION_KD    				164			//	(100/10000) * 16384
#define MAX_POSITION_LIMIT 			359
#define MIN_POSITION_LIMIT 			-359

#define HALL 						1
#define QEI_INDEX  					2
#define QEI_NO_INDEX				3

#define VELOCITY_FILTER_SIZE        8			//default (could be changed upto 16)

/**
 * \brief struct definition for PID Controller
 */
typedef struct S_Control
{
	int Kp_n, Kp_d; //Kp = Kp_n/Kp_d
	int Ki_n, Ki_d; //Ki = Ki_n/Ki_d
	int Kd_n, Kd_d; //Kd = Kd_n/Kd_d
	int Integral_limit;
	int Control_limit;
	int Loop_time;
} ctrl_par;

/**
 * \brief struct definition for velocity filter
 */
typedef struct S_Filter_length
{
	int filter_length;
} filter_par;


/**
 * \brief struct definition for quadrature sensor
 */
typedef struct S_QEI {
	int max_count;
	int real_counts;
	int gear_ratio;
	int index;   //no_index - 0 index - 1
	int poles;
} qei_par;


/**
 * \brief struct definition for hall sensor
 */
typedef struct S_Hall {
	int pole_pairs;
	int gear_ratio;
} hall_par;

/**
 * \brief struct definition for Synchronous torque param
 */
typedef struct CYCLIC_SYNCHRONOUS_TORQUE_PARAM
{
	int nominal_motor_speed;
	int nominal_current;
	int motor_torque_constant;
	int max_torque;
	int polarity;
} cst_par;

/**
 * \brief struct definition for Synchronous velocity param
 */
typedef struct CYCLIC_SYNCHRONOUS_VELOCITY_PARAM
{
	int max_motor_speed; // max motor speed
	int nominal_current;
	int motor_torque_constant;
	int polarity;
	int max_acceleration;
} csv_par;

/**
 * \brief struct definition for Synchronous position param
 */
typedef struct CYCLIC_SYNCHRONOUS_POSITION_PARAM
{
	csv_par base;
	int max_following_error;
	int max_position_limit;
	int min_position_limit;
} csp_par;

/**
 * \brief struct definition for profile torque param
 */
typedef struct PROFILE_TORQUE_PARAM
{
	int profile_slope;
	int polarity;
} pt_par;

/**
 * \brief struct definition for profile velocity param
 */
typedef struct PROFILE_VELOCITY_PARAM
{
	int max_profile_velocity;
	int profile_acceleration;
	int profile_deceleration;
	int quick_stop_deceleration;
	int polarity;
} pv_par;

/**
 * \brief struct definition for profile position param
 */
typedef struct PROFILE_POSITION_PARAM
{
	pv_par base;
	int profile_velocity;
	int software_position_limit_min;
	int software_position_limit_max;
	int max_acceleration;
} pp_par;

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
