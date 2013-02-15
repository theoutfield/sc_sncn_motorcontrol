/*
 * comm_sine.h
 *
 *  Created on: 11.05.2012
 *      Author: mschwarz
 */

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "sine_lookup.h"

void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl);

void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl);

unsigned root_function(unsigned uSquareValue);

void sine_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff);

#define DC900

//static t_pwm_control pwm_ctrl;

extern short sine_third[];
extern short arctg_table[];
extern short SPACE_TABLE[];
#ifdef DC100
extern out port testport;
#endif



#define defParRpmMotorMax		3742
#define defParDefSpeedMax		4000
#define defParRPMreference		4000
#define defParAngleUser 		 560
#define defParAngleFromRPM 		 150
#define defParUmotBoost  		 100
#define defParUmotStart 		 120
#define defParSpeedKneeUmot 	3500
#define defParAngleCorrVal         1
#define defParAngleCorrMax		 300


#define defParRmsLimit			1500  // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff      4000


#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			   64
#define defParIntegralGain		   64

#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64




