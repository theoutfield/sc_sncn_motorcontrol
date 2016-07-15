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
#include <position_feedback_service.h>
#include <profile_control.h>

#include <xscope.h>
//#include <bldc_motor_config.h>
#include <mc_internal_constants.h>
#include <user_config.h>

void run_offset_tuning(ProfilerConfig profiler_config,
                       client interface MotorcontrolInterface i_motorcontrol,
                       client interface PositionVelocityCtrlInterface ?i_position_control,
                       client interface PositionFeedbackInterface ?i_position_feedback);
