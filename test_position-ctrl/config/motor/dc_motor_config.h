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

#ifndef __DC_MOTOR_CONFIG__H__test1
#define __DC_MOTOR_CONFIG__H__test1
#include <print.h>


#pragma once

/*
 * define Motor Specific Constants
 */
#define POLE_PAIRS  8
#define GEAR_RATIO  26
#define MAX_NOMINAL_SPEED  4000		// in rpm
#define MAX_NOMINAL_CURRENT  2		// in A
#define MAX_ACCELERATION   5000     // rpm/s
#define QEI_COUNT_MAX_REAL 4000		// Max count of Quadrature Encoder
#define POLARITY 1					// 1 / -1

#define QEI_WITH_INDEX		1
#define QEI_WITH_NO_INDEX 	0
#define QEI_SENSOR_TYPE  	QEI_WITH_INDEX//QEI_WITH_NO_INDEX

#define MAX_FOLLOWING_ERROR 0
#define MAX_POSITION_LIMIT 	359
#define MIN_POSITION_LIMIT -359

#define DC100_RESOLUTION 	740
#define DC900_RESOLUTION	264

typedef struct S_Control
{
	int Kp_n, Kp_d; //Kp = Kp_n/Kp_d
	int Ki_n, Ki_d; //Ki = Ki_n/Ki_d
	int Kd_n, Kd_d; //Kd = Kd_n/Kd_d
	int Integral_limit;
	int Control_limit;
	int Loop_time;
} ctrl_par;

typedef struct S_Filter_length
{
	int filter_length;
} filt_par;


/**
 * \brief struct definition for quadrature sensor
 */
typedef struct S_QEI {
	int max_count;
	int real_counts;
	int gear_ratio;
	int index;   //no_index - 0 index - 1
} qei_par;


/**
 * \brief struct definition for hall sensor
 */
typedef struct S_Hall {
	int pole_pairs;
	int gear_ratio;
} hall_par;

typedef struct CYCLIC_SYNCHRONOUS_VELOCITY_PARAM
{
	int max_motor_speed; // max motor speed
	int nominal_current;
	int motor_torque_constant;
	int polarity;
} csv_par;


typedef struct CYCLIC_SYNCHRONOUS_POSITION_PARAM
{
	csv_par base;
	int max_following_error;
	int max_position_limit;
	int min_position_limit;
} csp_par;


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

#endif
