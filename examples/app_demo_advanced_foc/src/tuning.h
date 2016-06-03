/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <motorcontrol_service.h>


void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol);
void demo_torque_control(interface MotorcontrolInterface client i_motorcontrol);


/** BREAK COMMANDS */
typedef enum BREAK_CMD
{
    // activating and deactivating the breaks
    LOCK   = (0), // powering off the breaks (leads to locked joint)
    UNLOCK = (1), // powering on the breaks  (free rotating joint)
} BREAK_CMD;
