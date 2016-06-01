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
#include <position_ctrl_service.h>
#include <profile_control.h>

#include <xscope.h>
//#include <bldc_motor_config.h>
#include <mc_internal_constants.h>
#include <user_config.h>

void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface PositionControlInterface client ?i_position_control);
