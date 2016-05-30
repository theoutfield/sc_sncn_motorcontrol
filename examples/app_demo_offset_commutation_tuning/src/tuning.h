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
//#include <profile_control.h>
#include <hall_service.h>
#include <biss_service.h>
#include <ams_service.h>
#include <position_service.h>

#include <xscope.h>
#include <mc_internal_constants.h>
#include <user_config.h>

interface TuningInterface {
    void tune(int voltage);
    void set_limit(int limit);
    void set_position(int position);
    void set_torque(int in_torque);
    void set_pole_pairs(int in_pole_pairs);
    int  set_sensor_offset(int in_offset);
    int  auto_offset();
};


void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol, interface TuningInterface client ?i_tuning);

[[combinable]]
void tuning_service(interface TuningInterface server i_tuning, interface MotorcontrolInterface client i_motorcontrol,
                    interface ADCInterface client ?i_adc, interface PositionControlInterface client ?i_position_control,
                    client interface PositionInterface i_position);
