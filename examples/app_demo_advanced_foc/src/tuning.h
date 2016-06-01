/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <motorcontrol_service.h>


void run_offset_tuning(int position_limit, interface MotorcontrolInterface client i_motorcontrol);

