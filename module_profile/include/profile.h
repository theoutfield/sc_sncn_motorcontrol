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
#include <motor_control_structures.h>
#include <position_feedback_service.h>
#include <xccompat.h>

/*Profile Velocity Quick Stop*/

/**
 * @brief Initialize Quick Stop Velocity Profile
 *
 * @param actual_velocity
 * @param quick_stop_deceleration defines the deceleration for quick stop profile
 *
 * @return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_velocity_profile(int actual_velocity, int quick_stop_deceleration);

/**
 * @brief Generate Quick Stop Velocity Profile
 *
 * @param step current step of the profile
 *
 * @return corresponding target velocity at the step input
 */
extern int quick_stop_velocity_profile_generate(int step);

/*Profile Velocity Mode*/

extern void init_velocity_profile_limits(int max_velocity, int max_acceleration, int max_deceleration);

/**
 * @brief Initialize Velocity Profile
 *
 * @param target_velocity
 * @param actual_velocity
 * @param acceleration for the velocity profile
 * @param deceleration for the velocity profile
 *
 * @return no. of steps for velocity profile : range [1 - steps]
 */
extern int init_velocity_profile(int target_velocity, int actual_velocity, int acceleration, int deceleration);

/**
 * @brief Generate Velocity Profile
 *
 * @param step current step of the profile
 *
 * @return corresponding target velocity at the step input
 */
extern int velocity_profile_generate(int step);

/*Profile Position Mode*/

/**
 * @brief Initialize Position Profile Limits
 *
 * @param max_acceleration for the position profile
 * @param max_velocity for the position profile
 * @param qei_config Incremental Encoder configuration
 * @param hall_config Hall Sensor configuration
 * @param biss_config BiSS Encoder configuration
 * @param sensor_select
 * @param max_position
 * @param min_position
 *
 */
extern void init_position_profile_limits(int max_acceleration, int max_velocity, int max_position, int min_position, int ticks_per_turn);

/**
 * @brief Initialize Position Profile
 *
 * @param target_position
 * @param actual_position
 * @param velocity for the position profile
 * @param acceleration for the position profile
 * @param deceleration for the position profile
 *
 * @return no. of steps for position profile : range [1 - steps]
 */
extern int init_position_profile(int target_position, int actual_position, int velocity, int acceleration, \
                                 int deceleration);

/**
 * @brief Generate Position Profile
 *
 * @param step current step of the profile
 *
 * @return corresponding target position at the step input
 */
extern int position_profile_generate(int step);

/*Profile Position Quick Stop*/

/**
 * @brief Initialize Quick Stop Position Profile
 *
 * @param actual_velocity
 * @param actual_position
 * @param max_deceleration defines the deceleration for quick stop profile
 *
 * @return no. of steps for quick stop profile : range [1 - steps]
 */
extern int init_quick_stop_position_profile(int actual_velocity, int actual_position, int deceleration) ;

/**
 * @brief Generate Quick Stop Position Profile
 *
 * @param steps current step of the profile
 *
 * @return corresponding target position at the step input
 */
extern int quick_stop_position_profile_generate(int steps);

extern void init_linear_profile_limits(int max_value, int polarity);
extern int get_linear_profile_polarity();
/**
 * @brief Initialize Linear Profile
 *
 * @param target_value
 * @param actual_value
 * @param acceleration for the Linear profile
 * @param deceleration for the Linear profile
 *
 * @return no. of steps for linear profile : range [1 - steps]
 */
extern int init_linear_profile(int target_value, int actual_value, int acceleration, int deceleration);

/**
 * @brief Generate Linear Profile
 *
 * @param step current step of the profile
 *
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

    PositionFeedbackConfig position_feedback_params;
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


/**
 * @brief Structure type to set the parameters of position reference profiler.
 */
typedef struct {
    double delta_T;
    double resolution;              //resolution    [ticks/mechanical_rotation]
    double v_max;                   //velocity      [rpm]
    double acceleration_max;        //maximum acceleration  [rpm/s]
    double deceleration_max;        //maximum deceleration  [rpm/s]
    double torque_rate_max;         //torque rate   [mNm/s]
} ProfilerParam;

/**
 * @brief sign function.
 * @param output, sign of the number
 * @param input, number
 */
int sign_function(float a);

/**
 * @brief updating the torque reference profiler
 *
 * @param   torque_ref, target torque
 * @param   torque_ref_in_k_1n, profiled torque calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   torque_control_loop, the execution cycle of torque controller (us)
 *
 * @return  profiled torque calculated for the next step
 */
double torque_profiler(double torque_ref, double torque_ref_in_k_1n, ProfilerParam profiler_param, int torque_control_loop);

/**
 * @brief updating the velocity reference profiler
 *
 * @param   velocity_ref, target velocity
 * @param   velocity_ref_in_k_1n, profiled velocity calculated in one step
 * @param   profiler_param, structure containing the profiler parameters
 * @param   velocity_control_loop, the execution cycle of velocity controller (us)
 *
 * @return  profiled velocity calculated for the next step
 */
double velocity_profiler(double velocity_ref, double velocity_ref_in_k_1n, double velocity_actual, ProfilerParam profiler_param, int position_control_loop);

/**
 * @brief updating the position reference profiler
 * @param   pos_target, target position
 * @param   pos_k_1n, profiled position calculated in one step ago
 * @param   pos_k_2n, profiled position calculated in two steps ago
 * @param   pos_actual, profiled position calculated in three steps ago
 * @param   pos_profiler_param parameters of the position reference profiler
 *
 * @return  profiled position calculated for the next step
 */
float pos_profiler(double pos_target, double pos_k_1n, double pos_k_2n, double pos_k_3n, ProfilerParam pos_profiler_param);
