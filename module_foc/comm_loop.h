/*
 * comm_loop.h
 *
 *  Created on: 11.05.2012
 *      Author: mschwarz
 *      modified orgler@tin.it 04/2013
 */
#pragma once

#define DC900

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "sine_table_big.h"
#include "dc_motor_config.h"
#include "hall_client.h"

// Shared memory structure for the foc
typedef struct {
	int iHallSpeed,iHallAngle,iHallPosition,iHallPinState;
	int iEncoderSpeed,iEncoderAngle,iEncoderPosition,iEncoderPinState;
	int iAnglePWM;
	int iUmotMotor;
	int iPwmOnOff;
	int a1,a2,a1_calib,a2_calib,adcTemperature1,adcTemperature2,adcVoltageSupply,adcDummy,adcExternPoti1,adcExternPoti2;
}t_foc_control;

void    function_FOC_CONTROL(t_foc_control& focx, chanend c_commutation);



void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_encoder, chanend c_pwm_ctrl );

void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_encoder, chanend c_pwm_ctrl );










