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

#ifndef __DC_MOTOR_CONFIG__H__
#define __DC_MOTOR_CONFIG__H__

#pragma once

#define POLE_PAIRS	8
#define GEAR_RATIO	156
#define MAX_NOMINAL_SPEED  4000   // in 1/min
#define MAX_NOMINAL_CURRENT  5    // in A
#define QEI_COUNT_MAX (1024 * 4)  // Max count of Quadrature Encoder


/*define control parameters for PI TORQUE controller*/
#define Torque_Kp_n 40                   // Kp = Kp_n/Kp_d
#define Torque_Kp_d 10
#define Torque_Ki_n 4					 // Ki = Ki_n/Ki_d
#define Torque_Ki_d 120
#define Torque_Integral_limit 10000
#define Max_torque_out 1200              // Max_Torque_out = Max Continuous torque * 264 / torque constant

/*define control closing time the controller*/
#define loop_timing 88 					//in USEC_FAST




/*optional PI controller parameters for field control*/
#define Field_Kp_n 25                    // Kp = Kp_n/Kp_d
#define Field_Kp_d 10
#define Field_Ki_n 2					 // Ki = Ki_n/Ki_d
#define Field_Ki_d 100
#define Field_Integral_limit 10000



typedef struct S_Hall {
	int pole_pairs;
} hall_par;



/* initialize control loop parameter struct*/
void init_params_struct_all(torq_par &tor, field_par &field, loop_par &loop);

/* initialize hall sensor */
void init_hall(hall_par &h_pole);

/* initialize QEI sensor */
void init_qei(qei_par &q_max);
#endif
