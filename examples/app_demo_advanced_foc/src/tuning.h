/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <motorcontrol_service.h>
#include <position_feedback_service.h>



/** BREAK COMMANDS */
typedef enum BREAK_CMD
{
    // activating and deactivating the breaks
    LOCK   = (0), // powering off the breaks (leads to locked joint)
    UNLOCK = (1), // powering on the breaks  (free rotating joint)
} BREAK_CMD;


interface TuningInterface {
    void set_limit(int limit);
};

void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol, client interface TuningInterface ?i_tuning);

void position_limiter(interface TuningInterface server i_tuning, client interface MotorcontrolInterface i_motorcontrol);
