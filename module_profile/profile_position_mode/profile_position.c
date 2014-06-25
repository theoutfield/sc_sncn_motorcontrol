/**
 * @file profile_position.c
 * @brief Profile Generation for Position
 * Implements position profile based on Linear Function with
 * Parabolic Blends.
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <profile.h>


struct {
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

    qei_par qei_params;
    hall_par hall_params;
    int sensor_used;
    float max_position;
    float min_position;

} profile_pos_params;

int rpm_to_ticks_qei(int rpm, qei_par qei_params)
{
    int ticks = (rpm * qei_params.real_counts)/60;
    return ticks;
}

int rpm_to_ticks_hall(int rpm, hall_par hall_params)
{
    int ticks = (rpm * hall_params.max_ticks_per_turn)/60;
    return ticks;
}

int rpm_to_ticks_sensor(int rpm, int max_ticks_per_turn)
{
    int ticks = (rpm * max_ticks_per_turn)/60;
    return ticks;
}

void init_position_profile_limits(int max_acceleration, int max_velocity, qei_par qei_params,
                                  hall_par hall_params, int sensor_select, int max_position, int min_position)
{
    profile_pos_params.qei_params = qei_params;
    profile_pos_params.hall_params = hall_params;
    profile_pos_params.max_position =  max_position;
    profile_pos_params.min_position = min_position;
    profile_pos_params.sensor_used = sensor_select;
    if (profile_pos_params.sensor_used == HALL) {
        profile_pos_params.max_acceleration =  rpm_to_ticks_hall(max_acceleration, hall_params);
        profile_pos_params.max_velocity = rpm_to_ticks_hall(max_velocity, hall_params);
    } else if (profile_pos_params.sensor_used == QEI) {
        profile_pos_params.max_acceleration =  rpm_to_ticks_qei(max_acceleration, qei_params);
        profile_pos_params.max_velocity = rpm_to_ticks_qei(max_velocity, qei_params);
    } else {
        //profile_pos_params.max_acceleration =  rpm_to_ticks_sensor(max_acceleration, max_ticks_per_turn);
        //profile_pos_params.max_velocity = rpm_to_ticks_sensor(max_velocity, max_ticks_per_turn);
    }
    profile_pos_params.limit_factor = 10;
}

int init_position_profile(int target_position, int actual_position, int velocity,
                          int acceleration, int deceleration)
{
    profile_pos_params.qf = (float) target_position;
    profile_pos_params.qi = (float) actual_position;

    if (profile_pos_params.qf > profile_pos_params.max_position)
        profile_pos_params.qf = profile_pos_params.max_position;
    else if (profile_pos_params.qf < profile_pos_params.min_position)
        profile_pos_params.qf = profile_pos_params.min_position;

    if (profile_pos_params.sensor_used == QEI) {
        profile_pos_params.vi = rpm_to_ticks_qei(velocity, profile_pos_params.qei_params);
        profile_pos_params.acc =  rpm_to_ticks_qei(acceleration, profile_pos_params.qei_params);
        profile_pos_params.dec =  rpm_to_ticks_qei(deceleration, profile_pos_params.qei_params);
    } else if (profile_pos_params.sensor_used == HALL) {
        profile_pos_params.vi = rpm_to_ticks_hall(velocity, profile_pos_params.hall_params);
        profile_pos_params.acc =  rpm_to_ticks_hall(acceleration, profile_pos_params.hall_params);
        profile_pos_params.dec =  rpm_to_ticks_hall(deceleration, profile_pos_params.hall_params);
    } else {
        //profile_pos_params.vi = rpm_to_ticks_sensor(velocity, max_ticks_per_turn);
        //profile_pos_params.acc =  rpm_to_ticks_sensor(acceleration, max_ticks_per_turn);
        //profile_pos_params.dec =  rpm_to_ticks_sensor(deceleration, max_ticks_per_turn);
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

int position_profile_generate(int step)
{
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

void __initialize_position_profile_limits(int max_acceleration, int max_velocity,
                                          int sensor_select, int max_position, int min_position,
                                          profile_position_param *profile_pos_params)
{
    //profile_pos_params.qei_params;compute
    //profile_pos_params.hall_params = hall_params;
    profile_pos_params->max_position =  max_position;
    profile_pos_params->min_position = min_position;
    profile_pos_params->sensor_used = sensor_select;
    if (profile_pos_params->sensor_used == HALL) {
        profile_pos_params->max_acceleration =  rpm_to_ticks_hall(max_acceleration , profile_pos_params->hall_params);
        profile_pos_params->max_velocity = rpm_to_ticks_hall(max_velocity, profile_pos_params->hall_params);
    } else if (profile_pos_params->sensor_used == QEI) {
        profile_pos_params->max_acceleration =  rpm_to_ticks_qei(max_acceleration , profile_pos_params->qei_params);
        profile_pos_params->max_velocity = rpm_to_ticks_qei(max_velocity, profile_pos_params->qei_params);
    } else {
        //profile_pos_params.max_acceleration =  rpm_to_ticks_sensor(max_acceleration , max_ticks_per_turn);
        //profile_pos_params.max_velocity = rpm_to_ticks_sensor(max_velocity, max_ticks_per_turn);
    }
    profile_pos_params->limit_factor = 10;
}
//for c only
/*void __initialize_position_profile_limits(int gear_ratio, int max_acceleration, int max_velocity, profile_position_param *profile_pos_params)
  {
  profile_pos_params->max_acceleration = (max_acceleration * 6)/ gear_ratio;
  profile_pos_params->max_velocity = (max_velocity * 6)/ gear_ratio ;
  profile_pos_params->gear_ratio = (float) gear_ratio;
  }*/

int __initialize_position_profile(int target_position, int actual_position, int velocity, int acceleration,
                                  int deceleration, profile_position_param *profile_pos_params)
{
    profile_pos_params->qf = (float) target_position;

    profile_pos_params->qi = (float) actual_position;

    if (profile_pos_params->qf > profile_pos_params->max_position) {
        profile_pos_params->qf = profile_pos_params->max_position;
    } else if (profile_pos_params->qf < profile_pos_params->min_position) {
        profile_pos_params->qf = profile_pos_params->min_position;
    }

    if (profile_pos_params->sensor_used == QEI) {
        profile_pos_params->vi = rpm_to_ticks_qei(velocity, profile_pos_params->qei_params);
        profile_pos_params->acc =  rpm_to_ticks_qei(acceleration, profile_pos_params->qei_params);
        profile_pos_params->dec =  rpm_to_ticks_qei(deceleration, profile_pos_params->qei_params);
    } else if (profile_pos_params->sensor_used == HALL) {
        profile_pos_params->vi = rpm_to_ticks_hall(velocity, profile_pos_params->hall_params);
        profile_pos_params->acc =  rpm_to_ticks_hall(acceleration, profile_pos_params->hall_params);
        profile_pos_params->dec =  rpm_to_ticks_hall(deceleration, profile_pos_params->hall_params);
    } else {
        //profile_pos_params->vi = rpm_to_ticks_sensor(velocity, max_ticks_per_turn);
        //profile_pos_params->acc =  rpm_to_ticks_sensor(acceleration, max_ticks_per_turn);
        //profile_pos_params->dec =  rpm_to_ticks_sensor(deceleration, max_ticks_per_turn);
    }

    if (profile_pos_params->vi > profile_pos_params->max_velocity) {
        profile_pos_params->vi = profile_pos_params->max_velocity;
    }


    /* Internal params */

    profile_pos_params->acc_too_low = 0;

    profile_pos_params->qid = 0.0f;

    /* leads to shorter blend times in the begining (if init condition != 0) non zero case - not yet considered */

    profile_pos_params->qfd = 0.0f;


    /* compute distance */

    profile_pos_params->total_distance = profile_pos_params->qf - profile_pos_params->qi;

    profile_pos_params->direction = 1;

    if (profile_pos_params->total_distance < 0) {
        profile_pos_params->total_distance = -profile_pos_params->total_distance;
        profile_pos_params->direction = -1;
    }

    if (profile_pos_params->acc > profile_pos_params->limit_factor * profile_pos_params->total_distance) {
        profile_pos_params->acc = profile_pos_params->limit_factor * profile_pos_params->total_distance;
    }

    if (profile_pos_params->acc > profile_pos_params->max_acceleration) {
        profile_pos_params->acc = profile_pos_params->max_acceleration;
    }

    if (profile_pos_params->dec > profile_pos_params->limit_factor * profile_pos_params->total_distance) {
        profile_pos_params->dec = profile_pos_params->limit_factor * profile_pos_params->total_distance;
    }

    if (profile_pos_params->dec > profile_pos_params->max_acceleration) {
        profile_pos_params->dec = profile_pos_params->max_acceleration;
    }

    profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

    profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

    profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc *
                                        profile_pos_params->tb_acc) / 2.0f;

    profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec *
                                        profile_pos_params->tb_dec) / 2.0f;

    profile_pos_params->distance_left = (profile_pos_params->total_distance -
                                         profile_pos_params->distance_acc - profile_pos_params->distance_dec);


    /*check velocity and distance constraint*/

    if (profile_pos_params->distance_left < 0) {
        profile_pos_params->acc_too_low = 1;

        /* acc too low to meet distance/vel constraint */

        if (profile_pos_params->vi > profile_pos_params->total_distance) {
            profile_pos_params->vi = profile_pos_params->total_distance;

            profile_pos_params->acc_min = profile_pos_params->vi;

            if (profile_pos_params->acc < profile_pos_params->acc_min) {
                profile_pos_params->acc = profile_pos_params->acc_min;
            }

            if (profile_pos_params->dec < profile_pos_params->acc_min) {
                profile_pos_params->dec = profile_pos_params->acc_min;
            }
        } else if(profile_pos_params->vi < profile_pos_params->total_distance) {
            profile_pos_params->acc_min = profile_pos_params->vi;

            if (profile_pos_params->acc < profile_pos_params->acc_min) {
                profile_pos_params->acc = profile_pos_params->acc_min;
            }

            if (profile_pos_params->dec < profile_pos_params->acc_min) {
                profile_pos_params->dec = profile_pos_params->acc_min;
            }
        }

        profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

        profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

        profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc
                                            * profile_pos_params->tb_acc)/2.0f;

        profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec
                                            * profile_pos_params->tb_dec)/2.0f;

        profile_pos_params->distance_left = (profile_pos_params->total_distance -
                                             profile_pos_params->distance_acc - profile_pos_params->distance_dec);
    } else if (profile_pos_params->distance_left > 0) {
        profile_pos_params->acc_too_low = 0;
    }


    /* check velocity and min acceleration constraint */

    if (profile_pos_params->distance_left < 0) {
        profile_pos_params->acc_too_low = 1;

        /* acc too low to meet distance/velocity constraint */

        profile_pos_params->acc_min = profile_pos_params->vi;

        if(profile_pos_params->acc < profile_pos_params->acc_min) {
            profile_pos_params->acc = profile_pos_params->acc_min;
        }

        if(profile_pos_params->dec < profile_pos_params->acc_min) {
            profile_pos_params->dec = profile_pos_params->acc_min;
        }

        profile_pos_params->tb_acc = profile_pos_params->vi / profile_pos_params->acc;

        profile_pos_params->tb_dec = profile_pos_params->vi / profile_pos_params->dec;

        profile_pos_params->distance_acc = (profile_pos_params->acc * profile_pos_params->tb_acc *
                                            profile_pos_params->tb_acc)/2.0f;

        profile_pos_params->distance_dec = (profile_pos_params->dec * profile_pos_params->tb_dec *
                                            profile_pos_params->tb_dec)/2.0f;

        profile_pos_params->distance_left = (profile_pos_params->total_distance -
                                             profile_pos_params->distance_acc - profile_pos_params->distance_dec);
    } else if (profile_pos_params->distance_left > 0) {
        profile_pos_params->acc_too_low = 0;
    }

    profile_pos_params->distance_cruise = profile_pos_params->distance_left;

    profile_pos_params->t_cruise = (profile_pos_params->distance_cruise) / profile_pos_params->vi;

    profile_pos_params->tf = (profile_pos_params->tb_acc +
                              profile_pos_params->tb_dec + profile_pos_params->t_cruise);

    if (profile_pos_params->direction == -1) {
        profile_pos_params->vi = -profile_pos_params->vi;
    }

    /* compute LFPB motion constants */

    profile_pos_params->ai = profile_pos_params->qi;

    profile_pos_params->bi = profile_pos_params->qid;

    profile_pos_params->ci = (profile_pos_params->vi - profile_pos_params->qid) / (2.0f * profile_pos_params->tb_acc);

    profile_pos_params->di = profile_pos_params->ai + profile_pos_params->tb_acc * profile_pos_params->bi
        + profile_pos_params->ci * profile_pos_params->tb_acc * profile_pos_params->tb_acc
        - profile_pos_params->vi * profile_pos_params->tb_acc;

    profile_pos_params->ei = profile_pos_params->qf;

    profile_pos_params->fi = profile_pos_params->qfd;

    profile_pos_params->gi = (profile_pos_params->di + (profile_pos_params->tf - profile_pos_params->tb_dec) *
                              profile_pos_params->vi + profile_pos_params->fi * profile_pos_params->tb_dec -
                              profile_pos_params->ei) / (profile_pos_params->tb_dec * profile_pos_params->tb_dec);

    profile_pos_params->T = profile_pos_params->tf / 0.001f;  // 1 ms

    profile_pos_params->s_time = 0.001f;                      // 1 ms

    return (int) round(profile_pos_params->T);
}

//c only
int __position_profile_generate_in_steps(int step, profile_position_param *profile_pos_params)
{
    profile_pos_params->ts = profile_pos_params->s_time * step ;

    if (profile_pos_params->ts < profile_pos_params->tb_acc) {
        profile_pos_params->q = (profile_pos_params->ai + profile_pos_params->ts * profile_pos_params->bi +
                                 profile_pos_params->ci * profile_pos_params->ts * profile_pos_params->ts);
    } else if ( (profile_pos_params->tb_acc <= profile_pos_params->ts) &&
                (profile_pos_params->ts < (profile_pos_params->tf - profile_pos_params->tb_dec)) ) {
        profile_pos_params->q = profile_pos_params->di + profile_pos_params->vi * profile_pos_params->ts;
    } else if ( ((profile_pos_params->tf - profile_pos_params->tb_dec) <= profile_pos_params->ts) &&
                (profile_pos_params->ts <= profile_pos_params->tf) ) {
        profile_pos_params->q = (profile_pos_params->ei + (profile_pos_params->ts - profile_pos_params->tf) *
                                 profile_pos_params->fi + (profile_pos_params->ts - profile_pos_params->tf) *
                                 (profile_pos_params->ts - profile_pos_params->tf) * profile_pos_params->gi);
    }

    return (int) round(profile_pos_params->q);
}
