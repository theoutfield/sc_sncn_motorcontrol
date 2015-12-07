/**
 * @file internal_config.h
 * @brief Internal Definitions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

/*
 * @brief Modes of Operation (CiA402 Standard)
 *
 *  [M] - Mandatory
 *  [C] - Conditional
 *  [R] - Recommended
 *  [O] - optional
 *  [FG] - Function Group
 */

#define PP      1   /* [O] Profile Position mode */
#define VL      2   /* [O] Velocity mode (frequency converter) */
#define PV      3   /* [O] Profile velocity mode */
#define TQ      4   /* [O] Torque profile mode */
#define HM      6   /* [O] Homing mode */
#define IP      7   /* [O] Interpolated position mode */
#define CSP     8   /* [C] Cyclic synchronous position mode */
#define CSV     9   /* [C] Cyclic synchronous velocity mode */
#define CST     10  /* [C] Cyclic synchronous torque mode */
#define CSTCA   11  /* [O] Cyclic synchronous torque mode with commutation angle */

/**
 * @brief struct definition for Synchronous torque param
 */
typedef struct CYCLIC_SYNCHRONOUS_TORQUE_PARAM
{
    int nominal_motor_speed;
    int nominal_current;
    int motor_torque_constant;
    int max_torque;
    int polarity;
} cst_par;

/**
 * @brief struct definition for Synchronous velocity param
 */
typedef struct CYCLIC_SYNCHRONOUS_VELOCITY_PARAM
{
    int max_motor_speed;
    int nominal_current;
    int motor_torque_constant;
    int polarity;
    int max_acceleration;
} csv_par;

/**
 * @brief struct definition for Synchronous position param
 */

typedef struct CYCLIC_SYNCHRONOUS_POSITION_PARAM
{
    csv_par base;
    int max_following_error;
    int max_position_limit;
    int min_position_limit;
} csp_par;

/**
 * @brief struct definition for profile torque param
 */
typedef struct PROFILE_TORQUE_PARAM
{
    int profile_slope;
    int polarity;
} pt_par;

/**
 * @brief struct definition for profile velocity param
 */
typedef struct PROFILE_VELOCITY_PARAM
{
    int max_profile_velocity;
    int profile_acceleration;
    int profile_deceleration;
    int quick_stop_deceleration;
    int polarity;
} pv_par;

/**
 * @brief struct definition for profile position param
 */
typedef struct PROFILE_POSITION_PARAM
{
    pv_par base;
    int profile_velocity;
    int software_position_limit_min;
    int software_position_limit_max;
    int max_acceleration;
} pp_par;

