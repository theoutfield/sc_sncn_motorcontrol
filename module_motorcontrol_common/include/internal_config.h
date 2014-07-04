/**
 * @file internal_config.h
 * @brief Internal Definitions
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#pragma once

// default internal definitions (do not change)
#define BLDC_PWM_CONTROL_LIMIT              6794
#define BDC_PWM_CONTROL_LIMIT               13589

#define SET                                 1
#define UNSET                               0

#define HALL                                1
#define QEI                                 2
#define QEI_1                               3

#define QEI_WITH_INDEX                      1
#define QEI_WITH_NO_INDEX                   0

#define DC100_RESOLUTION                    740
#define DC300_RESOLUTION                    400
#define OLD_DC300_RESOLUTION                264

#define STAR_WINDING                        1
#define DELTA_WINDING                       2

#define INIT_BUSY                           0
#define INIT                                1
#define CHECK_BUSY                          10

#define SET_COMM_PARAM_ECAT                 20
#define SET_HALL_PARAM_ECAT                 21
#define SET_QEI_PARAM_ECAT                  22
#define SET_POSITION_CTRL_HALL              23
#define SET_POSITION_CTRL_QEI               24

#define SET_VELOCITY_CTRL_HALL              25
#define SET_VELOCITY_CTRL_QEI               26
#define INIT_VELOCITY_CTRL                  29
#define SET_VELOCITY_FILTER                 30
#define FILTER_SIZE                         8   // default
#define FILTER_SIZE_MAX                     128 // max size
#define SET_VELOCITY_TOKEN                  50
#define GET_VELOCITY_TOKEN                  60
#define VELOCITY_CTRL_STATUS                71
#define SHUTDOWN_VELOCITY_CTRL              200
#define ENABLE_VELOCITY_CTRL                250

#define SET_TORQUE_CTRL_HALL                27
#define SET_TORQUE_CTRL_QEI                 28
#define SET_TORQUE_TOKEN                    40
#define GET_TORQUE_TOKEN                    41
#define TORQUE_CTRL_STATUS                  71
#define SHUTDOWN_TORQUE_CTRL                201
#define ENABLE_TORQUE_CTRL                  251

#define SET_POSITION_TOKEN                  40
#define GET_POSITION_TOKEN                  41
#define HALL_PRECISION                      2
#define QEI_PRECISION                       512
#define POSITION_CTRL_STATUS                71
#define SHUTDOWN_POSITION_CTRL              201
#define ENABLE_POSITION_CTRL                251

#define SET_CTRL_PARAMETER                  100
#define SENSOR_SELECT                       150

#define ACTIVE_HIGH                         1
#define ACTIVE_LOW                          2

#define HOMING_NEGATIVE_SWITCH              1
#define HOMING_POSITIVE_SWITCH              2

#define SUCCESS                             1
#define ERROR                               0

#define NORMAL                              1
#define INVERTED                            -1
#define HALL_POSITION_INTERPOLATED_RANGE    4096

#define CONFIG_DIO_INPUT                    10
#define CONFIG_DIO_DONE                     15
#define GPIO_INPUT                          20
#define GPIO_OUTPUT                         22

/* Digital Input types */
#define GP_INPUT_TYPE                       40
#define SWITCH_INPUT_TYPE                   50

/* TODO: when / where / why are those used? */
#define COMMUTATION_FORWARD_CONSTANT        683     // 60  deg
#define COMMUTATION_REVERSE_CONSTANT        2731    // 240 deg

/**
 * @brief struct definition for PID Controller
 */
typedef struct S_Control
{
    int Kp_n, Kp_d; //Kp = Kp_n/Kp_d
    int Ki_n, Ki_d; //Ki = Ki_n/Ki_d
    int Kd_n, Kd_d; //Kd = Kd_n/Kd_d
    int Integral_limit;
    int Control_limit;
    int Loop_time;
} ctrl_par;

/**
 * @brief struct definition for velocity filter
 */
typedef struct S_Filter_length
{
    int filter_length;
} filter_par;

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
