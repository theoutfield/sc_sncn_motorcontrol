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
#include <motion_control_service.h>



/**
 * @brief Console app to tune the motor/motion control
 *
 * @param i_motion_control client interface to the motion control service
 *
 */
void control_tuning_console(client interface MotionControlInterface i_motion_control);
