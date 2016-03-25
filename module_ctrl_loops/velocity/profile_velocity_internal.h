/*
 * profile_velocity_internal.h
 *
 *  Created on: Mar 24, 2016
 *      Author: Synapticon
 */

#pragma once

typedef struct
{
    float max_acceleration, max_deceleration;  // max allowed acceleration & deceleration
    float max_velocity;                        // max allowed velocity
    float acc, dec;                            // acceleration & deceleration input
    float u;                                   // initial velocity
    float v_d;                                 // desired velocity
    float a_d;                                 // desired acceleration
    float t;                                   // time
    float T;                                   // total no. of Samples
    float s_time;                              // sampling time
    int oldst;                                 // old state of acc/dec
} ProfileVelocityParams;

int calculate_profile_steps(ProfileVelocityParams & profile_velocity_params);
int generate_profile_step_velocity(ProfileVelocityParams profile_velocity_params, int step);
