/**
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-02-22 1850)
 * \
 */

#ifndef __DC_MOTOR_CONFIG__H__
#define __DC_MOTOR_CONFIG__H__

#pragma once

// "Please define your the motor specs here"
// module_ctrl_loops initialisation needs to called to set up
// these variables for various control modules "else operation is not guaranteed"
// You still need to tune the control params for your motor individually

#define POLE_PAIRS	8
#define GEAR_RATIO	156
#define MAX_NOMINAL_SPEED  4000   // in 1/min
#define MAX_NOMINAL_CURRENT  5    // in A
#define QEI_COUNT_MAX (1024 * 4)  //

//PI controller ... and with intregal limit setting options.. also options to have PID (no default values used)
//#define Kp,Ki ..for torque control... field optional provide option to tune
//#define max torque level limit
//#provide option for loop timing only for torque control loop pwm is fixed to 18khz
//
//#define KP_torque  KI_TORQUE INTEGRAL_LIMIT_TORQUE MAX_TORQUE  (mN) convert upto user
//looping timing parameter
//optional not recommended opgtionla field tuning//

/*define control parameters for PI TORQUE controller*/
#define Torque_Kp_n 40                   // Kp = Kp_n/Kp_d
#define Torque_Kp_d 10
#define Torque_Ki_n 4					 // Ki = Ki_n/Ki_d
#define Torque_Ki_d 120
#define Torque_Integral_limit 10000
#define Max_torque_out 1200              //

/*define control closing time the controller*/
#define loop_timing 88 					//in USEC_FAST




/*optional controller parameters*/
#define Field_Kp_n 25                    // Kp = Kp_n/Kp_d
#define Field_Kp_d 10
#define Field_Ki_n 2					 // Ki = Ki_n/Ki_d
#define Field_Ki_d 100
#define Field_Integral_limit 10000


typedef struct S_Torque {
	int Kp_n, Kp_d;    					//Kp = Kp_n/Kp_d
	int Ki_n, Ki_d;						//Ki = Ki_n/Ki_d
	int Integral_limit;
	int Max_torque;
} torq_par;

typedef struct S_Loop_time {
	int delay;
} loop_par;

//optional control parameters
typedef struct S_Field{
	int Kp_n, Kp_d;						//Kp = Kp_n/Kp_d
	int Ki_n, Ki_d;						//Ki = Ki_n/Ki_d
	int Integral_limit;
} field_par;


typedef struct S_Hall {
	int pole_pairs;
} hall_par;
typedef struct S_QEI {
	unsigned max_count;
} qei_par;

void init_params_struct_all(torq_par &tor, field_par &field, loop_par &loop);


void init_hall(hall_par &h_pole);

/* initialize max QEI counts */
void init_qei(qei_par &q_max);
#endif
