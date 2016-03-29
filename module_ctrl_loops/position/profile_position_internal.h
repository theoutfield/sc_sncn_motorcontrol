/*
 * profile_velocity_internal.h
 *
 *  Created on: Mar 24, 2016
 *      Author: Synapticon
 */

#pragma once

#include <hall_service.h>
#include <qei_service.h>
#include <biss_service.h>
#include <ams_service.h>

typedef struct {
    float max_acceleration;   // max acceleration
    float max_velocity;

    /*User Inputs*/

    float acc;                // acceleration
    float dec;                // deceleration
    float vi;                 // velocity
    float qi;                 // initial position
    float qf;                 // final position

    /*Profile time*/

    float T;                  // total no. of Samples
    float s_time;             // sampling time

    int direction;
    int acc_too_low;          // flag for low acceleration constraint
    float acc_min;            // constraint minimum acceleration
    float limit_factor;       // max acceleration constraint

    /*LFPB motion profile constants*/

    float ai;
    float bi;
    float ci;
    float di;
    float ei;
    float fi;
    float gi;

    /*internal velocity variables*/

    float qid;                // initial velocity
    float qfd;                // final velocity

    float distance_cruise;
    float total_distance;
    float distance_acc;       // distance covered during acceleration
    float distance_dec;       // distance covered during deceleration
    float distance_left;      // distance left for cruise velocity

    float tb_acc;             // blend time for acceleration profile
    float tb_dec;             // blend time for deceleration profile
    float tf;                 // total time for profile
    float t_cruise;           // time for cruise velocity profile
    float ts;                 // variable to hold current sample time

    float q;                  // position profile

    QEIConfig qei_params;
    HallConfig hall_config;
    BISSConfig biss_config;
    AMSConfig ams_config;
    int sensor_used;
    float max_position;
    float min_position;

} ProfilePositionParams;

// Converter functions
int position_internal_rpm_to_ticks_qei(int rpm, QEIConfig qei_params);
int position_internal_rpm_to_ticks_hall(int rpm, HallConfig hall_config);
int position_internal_rpm_to_ticks_biss(int rpm, BISSConfig biss_config);
int position_internal_rpm_to_ticks_ams(int rpm, AMSConfig ams_config);
int position_internal_rpm_to_ticks_sensor(int rpm, int max_ticks_per_turn);

/**
 * @brief Calculate the number of steps for position profile
 *
 * @param profile_pos_params all parameters needed to do the calculations
 *
 * @return no. of steps for position profile : range [1 - steps]
 */
int calculate_profile_position_steps(ProfilePositionParams & profile_pos_params);

/**
 * @brief Generate Position Profile
 *
 * @param profile_pos_params all parameters needed to do the calculations
 * @param step current step of the profile
 *
 * @return corresponding target position at the step input
 */
int generate_profile_step_position(ProfilePositionParams & profile_pos_params, int step);
