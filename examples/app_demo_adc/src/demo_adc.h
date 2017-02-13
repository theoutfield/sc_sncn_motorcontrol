/*
 * demo_adc.h
 *
 *  Created on: Jul 13, 20117
 *      Author: Synapticon GmbH
 */


#pragma once

#include <platform.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>
#include <advanced_motorcontrol_licence.h>
#include <refclk.h>
#include <adc_service.h>
#include <position_ctrl_service.h>

#include <xscope.h>
//#include <bldc_motor_config.h>
#include <mc_internal_constants.h>
#include <user_config.h>

void demo_ad7265(interface ADCInterface client i_adc);
void demo_ad7949(interface ADCInterface client i_adc);
