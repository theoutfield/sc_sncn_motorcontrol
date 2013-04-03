
/**
 * \file comm_loop.h
 *
 *	Commutation rountine based on Space Vector PWM method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			& Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "sine_lookup.h"
#include "predriver/a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"

void comm_sine_init(chanend c_pwm_ctrl);

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue);

void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl,chanend c_motvalue);

unsigned root_function(unsigned uSquareValue);

//NEW!

void comm_sine_init_new(chanend c_pwm_ctrl);

void comm_sine_new(chanend c_value, chanend c_pwm_ctrl);

void commutation_new(chanend c_value, chanend c_pwm_ctrl, chanend sig);

void space_vector_pwm( int iIndexPWM, int iUmotMotor, int iMotHoldingTorque , t_pwm_control& pwm_ctrl, chanend c_pwm_ctrl, int iPwmOnOff );

//end of NEW!

#define DC900

//static t_pwm_control pwm_ctrl;

//extern short arctg_table[];
#ifdef DC100
extern out port testport;
#endif

