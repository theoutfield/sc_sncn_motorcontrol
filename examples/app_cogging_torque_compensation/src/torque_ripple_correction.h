#pragma once


#include <refclk.h>
#include <motion_control_service.h>
#include <tuning.h>
#include <xscope.h>
//#include <mc_internal_constants.h>
#include <position_feedback_service.h>
#include <stdlib.h>

#include <print.h>
#include <user_config.h>

#define SENSOR_RESOLUTION SENSOR_1_RESOLUTION

#define MEASURE_PRECISION 9
#define STEPS_PER_ROTATION 2*MOTOR_POLE_PAIRS*MEASURE_PRECISION


int interpolate_sensor_torque (int sensor_position);

void test_interpolate();
void map_torque_ripples(client interface MotionControlInterface i_motion_control, client interface PositionFeedbackInterface i_position_feedback
        , client interface TorqueControlInterface i_torque_control);

extern int cogging_torque [2][STEPS_PER_ROTATION];

void compensate_torque_ripples(client interface MotionControlInterface i_motion_control, client interface PositionFeedbackInterface i_position_feedback
        , client interface TorqueControlInterface i_torque_control);

void find_friction_torque(client interface MotionControlInterface i_motion_control, client interface PositionFeedbackInterface i_position_feedback
        , client interface TorqueControlInterface i_torque_control);
