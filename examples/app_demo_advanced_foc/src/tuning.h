/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <motor_control_interfaces.h>

/** BRAKE COMMANDS */
typedef enum BRAKE_CMD
{
    // activating and deactivating the brake
    LOCK   = (0), // powering off the brake (leads to locked joint)
    UNLOCK = (1), // powering on the brake  (free rotating joint)
} BRAKE_CMD;


interface TuningInterface
{
    void set_limit(int limit);
};

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol, client interface TuningInterface ?i_tuning);
void position_limiter(interface TuningInterface server i_tuning, client interface MotorcontrolInterface i_motorcontrol);
void demo_torque_control(interface MotorcontrolInterface client i_motorcontrol);
