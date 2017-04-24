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
#include <refclk.h>
#include <adc_service.h>
#include <motion_control_service.h>

#include <xscope.h>
#include <mc_internal_constants.h>

interface PositionLimiterInterface {
    void set_limit(int limit);
    int get_limit();
};

void control_tuning_console(client interface MotionControlInterface i_motion_control);
