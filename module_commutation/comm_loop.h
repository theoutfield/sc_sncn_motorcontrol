/*
 * comm_sine.h
 *
 *  Created on: 11.05.2012
 *      Author: mschwarz
 */

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "sine_table_big.h"
#include "sine_lookup.h"

void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue);

void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl,chanend c_motvalue);

unsigned root_function(unsigned uSquareValue);

void sine_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff);

#define DC900

//static t_pwm_control pwm_ctrl;









