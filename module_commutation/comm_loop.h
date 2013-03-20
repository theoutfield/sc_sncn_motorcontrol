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
#include "predriver/a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"

void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend c_value, chanend c_pwm_ctrl);

void commutation(chanend c_value, chanend c_pwm_ctrl, chanend sig);

unsigned root_function(unsigned uSquareValue);

void sine_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff);

void space_vector_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff );

void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1, chanend c_value);
