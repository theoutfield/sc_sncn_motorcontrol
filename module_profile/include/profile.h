/**
 * @file profile.h
 * @brief Profile Generation for Position, Velocity and Torque
 *      Implements position profile based on Linear Function with
 *      Parabolic Blends, velocity profile and torque profiles are
 *      based on linear functions.
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <stdio.h>
#include <math.h>
#include <mc_internal_constants.h>
#include <hall_service.h>
#include <qei_service.h>
#include <biss_service.h>
#include <ams_service.h>
#include <xccompat.h>

/*Profile Velocity Quick Stop*/

/**
 * @brief Initialise Quick Stop Velocity Profile
 *
 * @Input
 * @param actual_velocity
 * @param quick_stop_deceleration defines the deceleration for quick stop profile
 *
 * @Output
 * @return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);

/**
 * @brief Generate Quick Stop Velocity Profile
 *
 * @Input
 * @param step current step of the profile
 *
 * @Output
 * @return corresponding target velocity at the step input
 */
extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/

extern void init_velocity_profile_limits(int max_velocity, int max_acceleration, int max_deceleration);

/**
 * @brief Initialise Velocity Profile
 *
 * @Input
 * @param target_velocity
 * @param actual_velocity
 * @param acceleration for the velocity profile
 * @param deceleration for the velocity profile
 * @param max_velocity for the velocity profile
 *
 * @Output
 * @return no. of steps for velocity profile : range [1 - steps]
 */
extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration);

/**
 * @brief Generate Velocity Profile
 *
 * @Input
 * @param step current step of the profile
 *
 * @Output
 * @return corresponding target velocity at the step input
 */
extern int velocity_profile_generate(int step);

/*Profile Position Mode*/

/**
 * @brief Initialise Position Profile Limits
 *
 * @Input
 * @param gear_ratio
 * @param max_acceleration for the position profile
 * @param max_velocity for the position profile
 *
 */
extern void init_position_profile_limits(int max_acceleration, int max_velocity, QEIConfig qei_params, \
                                         HallConfig hall_config, BISSConfig biss_config, AMSConfig ams_config, int sensor_select, int max_position, int min_position);

/**
 * @brief Initialise Position Profile
 *
 * @Input
 * @param target_position
 * @param actual_position
 * @param velocity for the position profile
 * @param acceleration for the position profile
 * @param deceleration for the position profile
 *
 * @Output
 * @return no. of steps for position profile : range [1 - steps]
 */
extern int init_position_profile(int target_position, int actual_position, int velocity, int acceleration, \
                                 int deceleration);

/**
 * @brief Generate Position Profile
 *
 * @Input
 * @param step current step of the profile
 *
 * @Output
 * @return corresponding target position at the step input
 */
extern int position_profile_generate(int step);

/*Profile Position Quick Stop*/

/**
 * @brief Initialise Quick Stop Position Profile
 *
 * @Input
 * @param actual_velocity
 * @param actual_position
 * @param max_acceleration defines the deceleration for quick stop profile
 *
 * @Output
 * @return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_position_profile(int actual_velocity, int actual_position, int max_deceleration) ;

/**
 * @brief Generate Quick Stop Position Profile
 *
 * @Input
 * @param step current step of the profile
 * @param actual_velocity
 *
 * @Output
 * @return corresponding target position at the step input
 */
extern int quick_stop_position_profile_generate(int steps, int actual_velocity);

extern void init_linear_profile_limits(int max_value, int polarity);
extern int get_linear_profile_polarity();
/**
 * @brief Initialise Linear Profile
 *
 * @Input
 * @param target_value
 * @param actual_value
 * @param acceleration for the Linear profile
 * @param deceleration for the Linear profile
 * @param max_value for the Linear profile
 *
 * @Output
 * @return no. of steps for linear profile : range [1 - steps]
 */
extern int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration);

/**
 * @brief Generate Linear Profile
 *
 * @Input
 * @param step current step of the profile
 *
 * @Output
 * @return corresponding target value at the step input
 */
extern int linear_profile_generate(int step);

typedef struct
{
    float max_acceleration;     // max acceleration
    float max_velocity;

    /* User Inputs */

    float acc;                  // acceleration
    float dec;                  // deceleration
    float vi;                   // velocity
    float qi;                   // initial position
    float qf;                   // final position

    /* Profile time */

    float T;                    // total no. of Samples
    float s_time;               // sampling time

    int direction;
    int acc_too_low;            // flag for low acceleration constraint
    float acc_min;              // constraint minimum acceleration
    float limit_factor;         // max acceleration constraint

    /*LFPB motion profile constants*/

    float ai;
    float bi;
    float ci;
    float di;
    float ei;
    float fi;
    float gi;

    /*internal velocity variables*/

    float qid;                  // initial velocity
    float qfd;                  // final velocity

    float distance_cruise;
    float total_distance;
    float distance_acc;         // distance covered during acceleration
    float distance_dec;         // distance covered during deceleration
    float distance_left;        // distance left for cruise velocity

    float tb_acc;               // blend time for acceleration profile
    float tb_dec;               // blend time for deceleration profile
    float tf;                   // total time for profile
    float t_cruise;             // time for cruise velocity profile
    float ts;                   // variable to hold current sample time

    float q;                    // position profile

    QEIConfig qei_params;
    HallConfig hall_params;
    BISSConfig biss_params;
    AMSConfig ams_params;
    int sensor_used;
    float max_position;
    float min_position;

}       profile_position_param;

void __initialize_position_profile_limits( int max_acceleration, int max_velocity,
                                           int sensor_select, int max_position, int min_position,
                                           REFERENCE_PARAM(profile_position_param, profile_pos_params) );

int __initialize_position_profile( int target_position, int actual_position, int velocity, int acceleration,
                                   int deceleration, REFERENCE_PARAM(profile_position_param, profile_pos_params) );

int __position_profile_generate_in_steps(int step, REFERENCE_PARAM(profile_position_param, profile_pos_params) );

typedef struct
{
    float max_acceleration, max_deceleration;   // max allowed acceleration & deceleration
    float acc, dec;                             // acceleration & deceleration input
    float u;                                    // initial value
    float v_d;                                  // desired value
    float a_d;                                  // desired acceleration
    float t;                                    // time
    float T;                                    // total no. of Samples
    float s_time;                               // sampling time
    int oldst;                                  // old state of acc/dec
} profile_linear_param;

int __init_linear_profile_float( float target_value, float actual_value, float acceleration,
                                 float deceleration, float max_value, REFERENCE_PARAM(profile_linear_param, profile_linear_params) );

float __linear_profile_generate_float( int step, REFERENCE_PARAM(profile_linear_param, profile_linear_params) );

typedef struct
{
    float max_acceleration, max_deceleration;   // max allowed acceleration & deceleration
    float acc, dec;                             // acceleration & deceleration input
    float u;                                    // initial velocity
    float v_d;                                  // desired velocity
    float a_d;                                  // desired acceleration
    float t;                                    // time
    float T;                                    // total no. of Samples
    float s_time;                               // sampling time
    int oldst;                                  // old state of acc/dec
} profile_velocity_param;

int __initialize_velocity_profile( int target_velocity, int actual_velocity, int acceleration,
                                   int deceleration, int max_velocity, REFERENCE_PARAM(profile_velocity_param, profile_velocity_params) );

int __velocity_profile_generate_in_steps( int step, REFERENCE_PARAM(profile_velocity_param, profile_velocity_params) );


