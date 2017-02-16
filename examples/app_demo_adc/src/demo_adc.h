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

#include <xscope.h>
#include <mc_internal_constants.h>
#include <user_config.h>

void adc7265_client_demo(interface ADCInterface client i_adc);
void adc7949_client_demo(interface ADCInterface client i_adc);
