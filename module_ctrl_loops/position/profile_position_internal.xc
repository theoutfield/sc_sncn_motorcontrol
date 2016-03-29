/*
 * profile_position_internal.xc
 *
 *  Created on: Mar 28, 2016
 *      Author: Synapticon
 */

#include "profile_position_internal.h"
#include <math.h>

int position_internal_rpm_to_ticks_qei(int rpm, QEIConfig qei_params)
{
    int ticks = (rpm * qei_params.ticks_resolution*QEI_CHANGES_PER_TICK)/60;
    return ticks;
}

int position_internal_rpm_to_ticks_hall(int rpm, HallConfig hall_config)
{
    int ticks = (rpm * hall_config.pole_pairs*HALL_TICKS_PER_ELECTRICAL_ROTATION)/60;
    return ticks;
}

int position_internal_rpm_to_ticks_biss(int rpm, BISSConfig biss_config)
{
    int ticks = (rpm * (1 << biss_config.singleturn_resolution))/60;
    return ticks;
}

int position_internal_rpm_to_ticks_ams(int rpm, AMSConfig ams_config)
{
    int ticks = (rpm * (1 << ams_config.resolution_bits))/60;
    return ticks;
}

int position_internal_rpm_to_ticks_sensor(int rpm, int max_ticks_per_turn)
{
    int ticks = (rpm * max_ticks_per_turn)/60;
    return ticks;
}

int calculate_profile_position_steps(ProfilePositionParams & profile_pos_params) {
    if (profile_pos_params.qf > profile_pos_params.max_position) {
        profile_pos_params.qf = profile_pos_params.max_position;
    } else if (profile_pos_params.qf < profile_pos_params.min_position) {
        profile_pos_params.qf = profile_pos_params.min_position;
    }

    if (profile_pos_params.sensor_used == QEI_SENSOR) {
        profile_pos_params.vi = position_internal_rpm_to_ticks_qei(profile_pos_params.vi, profile_pos_params.qei_params);
        profile_pos_params.acc =  position_internal_rpm_to_ticks_qei(profile_pos_params.acc, profile_pos_params.qei_params);
        profile_pos_params.dec =  position_internal_rpm_to_ticks_qei(profile_pos_params.dec, profile_pos_params.qei_params);
    } else if (profile_pos_params.sensor_used == HALL_SENSOR) {
        profile_pos_params.vi = position_internal_rpm_to_ticks_hall(profile_pos_params.vi, profile_pos_params.hall_config);
        profile_pos_params.acc =  position_internal_rpm_to_ticks_hall(profile_pos_params.acc, profile_pos_params.hall_config);
        profile_pos_params.dec =  position_internal_rpm_to_ticks_hall(profile_pos_params.dec, profile_pos_params.hall_config);
    } else if (profile_pos_params.sensor_used == BISS_SENSOR) {
        profile_pos_params.vi =  position_internal_rpm_to_ticks_biss(profile_pos_params.vi, profile_pos_params.biss_config);
        profile_pos_params.acc =  position_internal_rpm_to_ticks_biss(profile_pos_params.acc, profile_pos_params.biss_config);
        profile_pos_params.dec =  position_internal_rpm_to_ticks_biss(profile_pos_params.dec, profile_pos_params.biss_config);
    } else if (profile_pos_params.sensor_used == AMS_SENSOR) {
        profile_pos_params.vi =  position_internal_rpm_to_ticks_ams(profile_pos_params.vi, profile_pos_params.ams_config);
        profile_pos_params.acc =  position_internal_rpm_to_ticks_ams(profile_pos_params.acc, profile_pos_params.ams_config);
        profile_pos_params.dec =  position_internal_rpm_to_ticks_ams(profile_pos_params.dec, profile_pos_params.ams_config);
    } else {
        //profile_pos_params.vi = position_internal_rpm_to_ticks_sensor(velocity, max_ticks_per_turn);
        //profile_pos_params.acc =  position_internal_rpm_to_ticks_sensor(acceleration, max_ticks_per_turn);
        //profile_pos_params.dec =  position_internal_rpm_to_ticks_sensor(deceleration, max_ticks_per_turn);
    }

    if (profile_pos_params.vi > profile_pos_params.max_velocity) {
        profile_pos_params.vi = profile_pos_params.max_velocity;
    }


    // Internal params

    profile_pos_params.acc_too_low = 0;

    profile_pos_params.qid = 0.0f;

    // leads to shorter blend times in the begining (if init condition != 0) non zero case - not yet considered

    profile_pos_params.qfd = 0.0f;


    // compute distance

    profile_pos_params.total_distance = profile_pos_params.qf - profile_pos_params.qi;

    profile_pos_params.direction = 1;

    if (profile_pos_params.total_distance < 0) {
        profile_pos_params.total_distance = -profile_pos_params.total_distance;
        profile_pos_params.direction = -1;
    }

    if (profile_pos_params.acc > profile_pos_params.limit_factor * profile_pos_params.total_distance) {
        profile_pos_params.acc = profile_pos_params.limit_factor * profile_pos_params.total_distance;
    }

    if (profile_pos_params.acc > profile_pos_params.max_acceleration) {
        profile_pos_params.acc = profile_pos_params.max_acceleration;
    }

    if (profile_pos_params.dec > profile_pos_params.limit_factor * profile_pos_params.total_distance) {
        profile_pos_params.dec = profile_pos_params.limit_factor * profile_pos_params.total_distance;
    }

    if (profile_pos_params.dec > profile_pos_params.max_acceleration) {
        profile_pos_params.dec = profile_pos_params.max_acceleration;
    }

    profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.acc;

    profile_pos_params.tb_dec = profile_pos_params.vi / profile_pos_params.dec;

    profile_pos_params.distance_acc = (profile_pos_params.acc * profile_pos_params.tb_acc
                                       * profile_pos_params.tb_acc) / 2.0f;

    profile_pos_params.distance_dec = (profile_pos_params.dec * profile_pos_params.tb_dec
                                       * profile_pos_params.tb_dec) / 2.0f;

    profile_pos_params.distance_left = (profile_pos_params.total_distance -
                                        profile_pos_params.distance_acc - profile_pos_params.distance_dec);


    // check velocity and distance constraint

    if (profile_pos_params.distance_left < 0) {
        profile_pos_params.acc_too_low = 1;

        // acc too low to meet distance/vel constraint

        if (profile_pos_params.vi > profile_pos_params.total_distance) {
            profile_pos_params.vi = profile_pos_params.total_distance;

            profile_pos_params.acc_min = profile_pos_params.vi;

            if (profile_pos_params.acc < profile_pos_params.acc_min) {
                profile_pos_params.acc = profile_pos_params.acc_min;
            }

            if (profile_pos_params.dec < profile_pos_params.acc_min) {
                profile_pos_params.dec = profile_pos_params.acc_min;
            }
        } else if (profile_pos_params.vi < profile_pos_params.total_distance) {
            profile_pos_params.acc_min = profile_pos_params.vi;

            if (profile_pos_params.acc < profile_pos_params.acc_min) {
                profile_pos_params.acc = profile_pos_params.acc_min;
            }

            if (profile_pos_params.dec < profile_pos_params.acc_min) {
                profile_pos_params.dec = profile_pos_params.acc_min;
            }
        }

        profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.acc;

        profile_pos_params.tb_dec = profile_pos_params.vi / profile_pos_params.dec;

        profile_pos_params.distance_acc = (profile_pos_params.acc * profile_pos_params.tb_acc *
                                           profile_pos_params.tb_acc)/2.0f;

        profile_pos_params.distance_dec = (profile_pos_params.dec * profile_pos_params.tb_dec *
                                           profile_pos_params.tb_dec)/2.0f;

        profile_pos_params.distance_left = (profile_pos_params.total_distance -
                                            profile_pos_params.distance_acc - profile_pos_params.distance_dec);

    } else if (profile_pos_params.distance_left > 0) {
        profile_pos_params.acc_too_low = 0;
    }


    // check velocity and min acceleration constraint

    if (profile_pos_params.distance_left < 0) {
        profile_pos_params.acc_too_low = 1;

        // acc too low to meet distance/velocity constraint

        profile_pos_params.acc_min = profile_pos_params.vi;

        if (profile_pos_params.acc < profile_pos_params.acc_min) {
            profile_pos_params.acc = profile_pos_params.acc_min;
        }

        if (profile_pos_params.dec < profile_pos_params.acc_min) {
            profile_pos_params.dec = profile_pos_params.acc_min;
        }

        profile_pos_params.tb_acc = profile_pos_params.vi / profile_pos_params.acc;

        profile_pos_params.tb_dec = profile_pos_params.vi / profile_pos_params.dec;

        profile_pos_params.distance_acc = (profile_pos_params.acc * profile_pos_params.tb_acc
                                           * profile_pos_params.tb_acc)/2.0f;

        profile_pos_params.distance_dec = (profile_pos_params.dec * profile_pos_params.tb_dec
                                           * profile_pos_params.tb_dec)/2.0f;

        profile_pos_params.distance_left = (profile_pos_params.total_distance -
                                            profile_pos_params.distance_acc - profile_pos_params.distance_dec);
    } else if (profile_pos_params.distance_left > 0) {
        profile_pos_params.acc_too_low = 0;
    }

    profile_pos_params.distance_cruise = profile_pos_params.distance_left;

    profile_pos_params.t_cruise = (profile_pos_params.distance_cruise) / profile_pos_params.vi;

    profile_pos_params.tf = (profile_pos_params.tb_acc +
                             profile_pos_params.tb_dec + profile_pos_params.t_cruise);

    if (profile_pos_params.direction == -1) {
        profile_pos_params.vi = -profile_pos_params.vi;
    }

    // compute LFPB motion constants

    profile_pos_params.ai = profile_pos_params.qi;

    profile_pos_params.bi = profile_pos_params.qid;

    profile_pos_params.ci = (profile_pos_params.vi - profile_pos_params.qid) / (2.0f * profile_pos_params.tb_acc);

    profile_pos_params.di = (profile_pos_params.ai + profile_pos_params.tb_acc * profile_pos_params.bi +
                             profile_pos_params.ci * profile_pos_params.tb_acc * profile_pos_params.tb_acc -
                             profile_pos_params.vi * profile_pos_params.tb_acc);

    profile_pos_params.ei = profile_pos_params.qf;

    profile_pos_params.fi = profile_pos_params.qfd;

    profile_pos_params.gi = (profile_pos_params.di + (profile_pos_params.tf - profile_pos_params.tb_dec) *
                             profile_pos_params.vi + profile_pos_params.fi * profile_pos_params.tb_dec -
                             profile_pos_params.ei) / (profile_pos_params.tb_dec * profile_pos_params.tb_dec);

    profile_pos_params.T = profile_pos_params.tf / 0.001f;  // 1 ms

    profile_pos_params.s_time = 0.001f;                     // 1 ms

    return (int) round(profile_pos_params.T);
}

int generate_profile_step_position(ProfilePositionParams & profile_pos_params, int step) {
    profile_pos_params.ts = profile_pos_params.s_time * step ;

    if (profile_pos_params.ts < profile_pos_params.tb_acc) {

        profile_pos_params.q = (profile_pos_params.ai + profile_pos_params.ts * profile_pos_params.bi +
                                profile_pos_params.ci * profile_pos_params.ts * profile_pos_params.ts);

    } else if ( (profile_pos_params.tb_acc <= profile_pos_params.ts) &&
                (profile_pos_params.ts < (profile_pos_params.tf - profile_pos_params.tb_dec)) ) {

        profile_pos_params.q = profile_pos_params.di + profile_pos_params.vi * profile_pos_params.ts;

    } else if ( ((profile_pos_params.tf - profile_pos_params.tb_dec) <= profile_pos_params.ts) &&
                (profile_pos_params.ts <= profile_pos_params.tf) ) {

        profile_pos_params.q = (profile_pos_params.ei + (profile_pos_params.ts - profile_pos_params.tf) *
                                profile_pos_params.fi + (profile_pos_params.ts - profile_pos_params.tf) *
                                (profile_pos_params.ts - profile_pos_params.tf) * profile_pos_params.gi);
    }

    return (int) round(profile_pos_params.q);
}
