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
#include <position_feedback_service.h>

/**
 * @brief Console app to tune the motor/motion control
 *
 * @param i_motion_control      client interface to the motion control service
 * @param i_position_feedback_1 client interface to position feedback serivce 1
 * @param i_position_feedback_2 client interface to position feedback service 2
 */
void control_tuning_console(
        client interface MotionControlInterface i_motion_control,
        client interface PositionFeedbackInterface ?i_position_feedback_1,
        client interface PositionFeedbackInterface ?i_position_feedback_2);

/**
 * @brief App that checks motor-drive connections, and evaluates sensor behavior
 *
 * @param i_motion_control client interface to the motion control service
 *
 * return error if it exists
 */
int general_system_evaluation(client interface MotionControlInterface i_motion_control);
