/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
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

interface PositionLimiterInterface {
    void set_limit(int limit);
    int get_limit();
};

void demo_torque_position_velocity_control(interface MotorcontrolInterface client i_commutation, interface PositionVelocityCtrlInterface client ?i_position_control);
