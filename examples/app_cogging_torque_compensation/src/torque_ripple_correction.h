#pragma once


#include <refclk.h>
#include <motion_control_service.h>

#include <xscope.h>
//#include <mc_internal_constants.h>
#include <position_feedback_service.h>
#include <stdlib.h>

#include <print.h>
#include <user_config.h>

#define SENSOR_RESOLUTION SENSOR_1_RESOLUTION

void test_interpolate();
void map_torque_ripples(client interface PositionVelocityCtrlInterface i_position_control, client interface PositionFeedbackInterface i_position_feedback
        );
