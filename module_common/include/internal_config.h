
/**
 * \file internal_config.h
 * \brief Internal Definitions
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

#ifndef INTERNAL_CONFIG_H_
#define INTERNAL_CONFIG_H_
#pragma once

// default internal definitions (do not change)
#define CONTROL_LIMIT_PWM					6944
#define TORQUE_CTRL_READ(x)					c_torque_ctrl :> x
#define TORQUE_CTRL_WRITE(x)				c_torque_ctrl <: x

#define VELOCITY_CTRL_READ(x)				c_velocity_ctrl :> x
#define VELOCITY_CTRL_WRITE(x)				c_velocity_ctrl <: x

#define POSITION_CTRL_READ(x)				c_position_ctrl :> x
#define POSITION_CTRL_WRITE(x)				c_position_ctrl <: x

#define SIGNAL_READ(x) 						c_signal :> x
#define SIGNAL_WRITE(x)						c_signal <: x

#define SET    								1
#define UNSET  								0

#define HALL 								1
#define QEI 								2
#define QEI_1								3


#define QEI_WITH_INDEX						1
#define QEI_WITH_NO_INDEX 					0

#define DC100_RESOLUTION 					740
#define DC300_RESOLUTION					400
#define OLD_DC300_RESOLUTION				264

#define STAR_WINDING						1
#define DELTA_WINDING						2

#define INIT_BUSY 							0
#define INIT								1
#define CHECK_BUSY  						10

#define SET_COMM_PARAM_ECAT 				20
#define SET_HALL_PARAM_ECAT 				21
#define SET_QEI_PARAM_ECAT  				22
#define SET_POSITION_CTRL_HALL 				23
#define SET_POSITION_CTRL_QEI				24


#define SET_VELOCITY_CTRL_HALL 				25
#define SET_VELOCITY_CTRL_QEI				26
#define INIT_VELOCITY_CTRL  				29
#define SET_VELOCITY_FILTER 				30
#define FILTER_SIZE 						8          	// default
#define FILTER_SIZE_MAX 					128			// max size
#define SET_VELOCITY_TOKEN 					50
#define GET_VELOCITY_TOKEN 					60
#define VELOCITY_CTRL_STATUS				71
#define SHUTDOWN_VELOCITY_CTRL				200
#define ENABLE_VELOCITY_CTRL				250

#define SET_TORQUE_CTRL_HALL 				27
#define SET_TORQUE_CTRL_QEI  				28
#define SET_TORQUE_TOKEN 					40
#define GET_TORQUE_TOKEN 					41
#define TORQUE_CTRL_STATUS					71
#define SHUTDOWN_TORQUE_CTRL				201
#define ENABLE_TORQUE_CTRL					251

#define SET_POSITION_TOKEN 					40
#define GET_POSITION_TOKEN 					41
#define HALL_PRECISION						2
#define QEI_PRECISION						512
#define POSITION_CTRL_STATUS				71
#define SHUTDOWN_POSITION_CTRL				201
#define ENABLE_POSITION_CTRL				251

#define SET_VOLTAGE    						2
#define SET_COMMUTATION_PARAMS 				3
#define DISABLE_FETS						4
#define ENABLE_FETS							5
#define FETS_STATE							6

#define SET_CTRL_PARAMETER 					100
#define SENSOR_SELECT      					150

#define ACTIVE_HIGH							1
#define ACTIVE_LOW							2

#define HOMING_NEGATIVE_SWITCH				1
#define HOMING_POSITIVE_SWITCH				2

#define SUCCESS 							1
#define ERROR   							0

#define NORMAL								1
#define INVERTED							-1
#define HALL_POSITION_INTERPOLATED_RANGE 	4096


#define CONFIG_DIO_INPUT					10
#define CONFIG_DIO_DONE						15
#define GPIO_INPUT							20
#define GPIO_OUTPUT							22

/* Digital Input types */
#define GP_INPUT_TYPE						40
#define SWITCH_INPUT_TYPE					50

#define COMMUTATION_FORWARD_CONSTANT 		683  	// 60  deg
#define COMMUTATION_REVERSE_CONSTANT 		2731	// 240 deg

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
	int max_ticks_per_turn;
	int real_counts;	//	int gear_ratio;
	int max_ticks;	// paramater allows for more turns
	int index;   	// no_index - 0 index - 1
	int poles;
	int sensor_polarity;
} qei_par;

/**
 * \brief struct definition for hall sensor
 */
typedef struct S_Hall {
	int pole_pairs;
	int max_ticks_per_turn;
	int max_ticks;
	int sensor_polarity;
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
	int max_motor_speed;
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

#endif /* INTERNAL_CONFIG_H_ */
