/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <platform.h>
#include <motorcontrol_service.h>
#include <pwm_service.h>
#include <refclk.h>
#include <adc_service.h>

#include <xscope.h>
//#include <bldc_motor_config.h>
#include <mc_internal_constants.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface ADCInterface client ?i_adc);

int auto_tuning_current(interface MotorcontrolInterface client i_commutation, interface ADCInterface client i_adc, int input_voltage);
