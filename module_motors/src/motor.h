
/**
 * \file motor.h
 * \brief Initialized motor parameters. Please a desired motor with #include<your_motor.h>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#ifndef MOTOR_H_
#define MOTOR_H_
#include <print.h>
#include <internal_config.h>
#include <motor_maxon_313320.h>
#include <motor_generic_bldc.h>

#pragma once

<<<<<<< HEAD:module_motors/src/motor.h
#define IFM_RESOLUTION              DC100_RESOLUTION
#define VELOCITY_FILTER_SIZE        24      //default (could be changed upto 128)
=======
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

#define QEI_SENSOR_POLARITY			NORMAL		// NORMAL

/*Somanet IFM Internal Config*/
#define IFM_RESOLUTION				DC100_RESOLUTION  // DC300_RESOLUTION   /* Specifies the current sensor resolution/A

/*Changes direction of the motor drive*/
#define POLARITY 					1		// 1 / -1

/*Commutation offset (range 0-4095) */
#define COMMUTATION_OFFSET_CLK		683
#define COMMUTATION_OFFSET_CCLK		2731

/*Motor Winding type*/
#define WINDING_TYPE				DELTA_WINDING		// 1: star-type(Y) or 2: delta-type STAR_WINDING//

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
>>>>>>> master:test_ethercat-motorctrl-mode/config/motor/bldc_motor_config.h

/**
 * \brief Initialize Velocity sensor filter
 *
 * \param sensor_filter_par struct defines the velocity filter params
 */
void init_sensor_filter_param(filter_par &sensor_filter_par) ;

/**
 * \brief Initialize QEI sensor
 *
 * \param qei_params struct defines the resolution for quadrature encoder (QEI),
 *          gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par &qei_params);

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

/**
 * \brief Initialize cyclic synchronous velocity params
 *
 * \param csv_params struct defines cyclic synchronous velocity params
 */
void init_csv_param(csv_par &csv_params);

/**
 * \brief Initialize cyclic synchronous position params
 *
 * \param csp_params struct defines cyclic synchronous position params
 */
void init_csp_param(csp_par &csp_params);

/**
 * \brief Initialize cyclic synchronous torque params
 *
 * \param cst_params struct defines cyclic synchronous torque params
 */
void init_cst_param(cst_par &cst_params);

/**
 * \brief Initialize profile position params
 *
 * \param pp_params struct defines profileposition params
 */
void init_pp_params(pp_par &pp_params);

/**
 * \brief Initialize profile velocity params
 *
 * \param pv_params struct defines profile velocity params
 */
void init_pv_params(pv_par &pv_params);

/**
 * \brief Initialize profile torque params
 *
 * \param pt_params struct defines profile torque params
 */
void init_pt_params(pt_par &pt_params);

/**
 * \brief Initialize torque control PID params
 *
 * \param torque_ctrl_params struct defines torque control PID params
 */
void init_torque_control_param(ctrl_par &torque_ctrl_params);

/**
 * \brief Initialize velocity control PID params
 *
 * \param velocity_ctrl_params struct defines velocity control PID params
 */
void init_velocity_control_param(ctrl_par &velocity_ctrl_params);

/**
 * \brief Initialize position control PID params
 *
 * \param position_ctrl_params struct defines position control PID params
 */
void init_position_control_param(ctrl_par &position_ctrl_params);

#endif

