/**************************************************************************
 * \file dc_motor_config.h
 *	Motor Control config file
 *
 * Please define your the motor specifications here
 *
 * All these initialisation functions :init_params_struct_all, init_hall and init_qei
 * need to be called to set up the variables for control module, hall sensor and quadrature
 * sensor modules "else operation is not guaranteed"
 *
 * You still need to tune the PI torque control params for your motor individually
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com> & Martin Schwarz <mschwarz@synapticon.com>
 *
 * All code contained in this package under Synapticon copyright must be
 * licensing for any use from Synapticon. Please contact support@synapticon.com for
 * details of licensing.
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **************************************************************************/

#ifndef __DC_MOTOR_CONFIG__H__VELOCITY
#define __DC_MOTOR_CONFIG__H__VELOCITY
#include <print.h>
#include <internal_config.h>

#pragma once

/*
 * define Motor Specific Constants (may conform to CiA 402 Standards)
 */
#define POLE_PAIRS  8
#define GEAR_RATIO  26
#define MAX_NOMINAL_SPEED  5360		// in rpm
#define MAX_NOMINAL_CURRENT  2		// in A
#define MAX_ACCELERATION   5000     // rpm/s
#define QEI_COUNT_MAX_REAL 4000		// Max count of Quadrature Encoder
#define POLARITY 1					// 1 / -1

#define QEI_WITH_INDEX		1
#define QEI_WITH_NO_INDEX 	0
#define QEI_SENSOR_TYPE  	QEI_WITH_INDEX//QEI_WITH_NO_INDEX
#define SENSOR_PLACEMENT			OUT_OF_PHASE		// OUT_OF_PHASE

#define MAX_FOLLOWING_ERROR 0
#define MAX_POSITION_LIMIT 	359
#define MIN_POSITION_LIMIT -359

/*External Controller Configs*/
#define TORQUE_Kp_NUMERATOR 	   	50
#define TORQUE_Kp_DENOMINATOR  		10
#define TORQUE_Ki_NUMERATOR    		11
#define TORQUE_Ki_DENOMINATOR  		110
#define TORQUE_Kd_NUMERATOR    		1
#define TORQUE_Kd_DENOMINATOR  		10

#define VELOCITY_Kp_NUMERATOR 	 	5
#define VELOCITY_Kp_DENOMINATOR  	10
#define VELOCITY_Ki_NUMERATOR    	5
#define VELOCITY_Ki_DENOMINATOR  	100
#define VELOCITY_Kd_NUMERATOR    	0
#define VELOCITY_Kd_DENOMINATOR  	1


#define POSITION_Kp_NUMERATOR 	 	180
#define POSITION_Kp_DENOMINATOR  	2000
#define POSITION_Ki_NUMERATOR    	50
#define POSITION_Ki_DENOMINATOR  	102000
#define POSITION_Kd_NUMERATOR    	100
#define POSITION_Kd_DENOMINATOR  	10000

#define MAX_PROFILE_VELOCITY  		MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY			1001
#define PROFILE_ACCELERATION		2002
#define PROFILE_DECELERATION  		2004
#define QUICK_STOP_DECELERATION 	2005

/*Somanet IFM Internal Config*/
#define IFM_RESOLUTION		DC300_RESOLUTION

#define COMMUTATION_ANGLE_OFFSET_CLOCKWISE			  480  // range (0 - 4095)
#define COMMUTATION_ANGLE_OFFSET_COUNTERCLOCKWISE	 3000  // range (0 - 4095)



/**
 * \brief initialize QEI sensor
 *
 * \param q_max struct defines the max count for quadrature encoder (QEI)
 */
void init_qei_param(qei_par &qei_params);

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

void init_csv_param(csv_par &csv_params);

void init_csp_param(csp_par &csp_params);

void init_velocity_control_param(ctrl_par &velocity_ctrl_params);

void init_position_control_param(ctrl_par &position_ctrl_params);

void init_pp_params(pp_par &pp_params);

void init_pv_params(pv_par &pv_params);

void init_cst_param(cst_par &cst_params);

void init_torque_control_param(ctrl_par &torque_ctrl_params);
#endif
