/**
 * @file internal_config.h
 * @brief Internal Definitions
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <mc_constants.h>
#include <pwm_cli_common.h>

/* TODO: output of control loops shouldn't (directly) depend on
 * PWM_MAX_VALUE, etc. */
#define BLDC_PWM_CONTROL_LIMIT              (((PWM_MAX_VALUE) - (PWM_DEAD_TIME)) / 2)
#define BDC_PWM_CONTROL_LIMIT               ((PWM_MAX_VALUE) - (PWM_DEAD_TIME))

#define HALL_POSITION_INTERPOLATED_RANGE    4096

/* FIXME: those should be moved to board support packages */
#define DC100_RESOLUTION                    740
#define DC300_RESOLUTION                    400
#define OLD_DC300_RESOLUTION                264

#define BLDC                                1
#define BDC                                 2

#define MOTOR_TYPE                          BLDC

#define INIT_BUSY                           0
#define INIT                                1
#define CHECK_BUSY                          10

#define SENSOR_SELECT                       150

#define FILTER_SIZE                         8   // default (but used anywhere)
#define FILTER_SIZE_MAX                     128 // max size

#define SET_COMM_PARAM_ECAT                 20
#define set_hall_conifg_ecat                 21
#define SET_QEI_PARAM_ECAT                  22

#define INIT_VELOCITY_CTRL                  29
#define SET_VELOCITY_FILTER                 30

#define HALL_PRECISION                      2
#define QEI_PRECISION                       512

#define SET_CTRL_PARAMETER                  100

#define CONFIG_DIO_INPUT                    10
#define CONFIG_DIO_DONE                     15
#define GPIO_INPUT                          20
#define GPIO_OUTPUT                         22

/* Digital Input types */
#define GP_INPUT_TYPE                       40
#define SWITCH_INPUT_TYPE                   50

/**
 * @brief struct definition for velocity filter
 */
/*
typedef struct S_Filter_length
{
    int filter_length;
} filter_par;
*/
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
