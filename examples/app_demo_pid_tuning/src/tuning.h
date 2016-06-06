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

interface TuningInterface {
    void tune(int voltage);
    void set_limit(int limit);
    void set_position(int position);
    void set_position_direct(int position);
    void set_torque(int in_torque);
    void set_torque_limit(int in_torque_limit);
};


void run_offset_tuning(int input_voltage, interface MotorcontrolInterface client i_commutation, interface TuningInterface client ?i_tuning);

int auto_tuning_current(interface MotorcontrolInterface client i_commutation, interface ADCInterface client i_adc, int input_voltage);

[[combinable]]
void tuning_service(interface TuningInterface server i_tuning, interface MotorcontrolInterface client i_commutation,
                    interface ADCInterface client ?i_adc, interface PositionVelocityCtrlInterface client ?i_position_control,
                    interface HallInterface client ?i_hall, interface BISSInterface client ?i_biss, interface AMSInterface client ?i_ams);
