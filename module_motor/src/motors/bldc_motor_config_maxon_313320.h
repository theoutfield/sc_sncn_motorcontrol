/**
 * @file motor_maxon_313320.h
 * @brief Motor parameters for the maxon EC-i 40, brushless, 50 Watt motor with Hall sensors (Part Number: 313320)
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <internal_config.h>
#include <bldc_motor_init.h>

/**
 * Define Motor Specific Constants (found in specification sheet of your motor)
 * Mandatory constants to be set
 */
#define POLE_PAIRS                  8      // Number of pole pairs
#define MAX_NOMINAL_SPEED           5260   // rpm
#define MAX_NOMINAL_CURRENT         2      // A
#define MOTOR_TORQUE_CONSTANT       34     // mNm/A

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO                  1      // if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION          4000   // 4 x Max count of Incremental Encoder (4X decoding - quadrature mode)

/* Position/Velocity Sensor Types (select your sensor type here)
 * (HALL/ QEI) */
#define SENSOR_USED                 HALL

/* Define your Incremental Encoder type (QEI_WITH_INDEX/ QEI_NO_INDEX) */
#define QEI_SENSOR_TYPE             QEI_WITH_INDEX

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (QEI_POLARITY_NORMAL/QEI_POLARITY_INVERTED) */
#define QEI_SENSOR_POLARITY         QEI_POLARITY_NORMAL

/* Commutation offset (range 0-4095) (HALL sensor based commutation) */
#define COMMUTATION_OFFSET_CLK      683
#define COMMUTATION_OFFSET_CCLK     2731

/* Motor Winding type (STAR_WINDING/DELTA_WINDING) */
#define WINDING_TYPE                STAR_WINDING

/* Changes direction of the motor drive  (1 /-1) */
#define POLARITY                    1

#define MAX_POSITION_LIMIT          16000   // ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT          -16000  // ticks (min range: -2^30, limited for safe operation)

/* Profile defines (Mandatory for profile modes) */
#define MAX_PROFILE_VELOCITY        MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY            1001   // rpm
#define MAX_ACCELERATION            3000   // rpm/s
#define PROFILE_ACCELERATION        2002   // rpm/s
#define PROFILE_DECELERATION        2004   // rpm/s
#define QUICK_STOP_DECELERATION     2005   // rpm/s
#define PROFILE_TORQUE_SLOPE        66     // (desired torque_slope/torque_constant)  * IFM resolution

/* Control specific constants/variables */

/* Velocity Control (Mandatory if Velocity control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define VELOCITY_Kp_NUMERATOR       5
#define VELOCITY_Kp_DENOMINATOR     10
#define VELOCITY_Ki_NUMERATOR       5
#define VELOCITY_Ki_DENOMINATOR     100
#define VELOCITY_Kd_NUMERATOR       0
#define VELOCITY_Kd_DENOMINATOR     1

/* Torque Control (Mandatory if Torque control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define TORQUE_Kp_NUMERATOR         50
#define TORQUE_Kp_DENOMINATOR       10
#define TORQUE_Ki_NUMERATOR         11
#define TORQUE_Ki_DENOMINATOR       110
#define TORQUE_Kd_NUMERATOR         1
#define TORQUE_Kd_DENOMINATOR       10

/* Position Control (Mandatory if Position control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define POSITION_Kp_NUMERATOR       180
#define POSITION_Kp_DENOMINATOR     2000
#define POSITION_Ki_NUMERATOR       50
#define POSITION_Ki_DENOMINATOR     102000
#define POSITION_Kd_NUMERATOR       100
#define POSITION_Kd_DENOMINATOR     10000

#define VELOCITY_FILTER_SIZE        8   //default (could be changed upto 16)

/* Define Homing method (HOMING_POSITIVE_SWITCH/HOMING_NEGATIVE_SWITCH)
 * this specifies direction for the node to find the home switch */
#define HOMING_METHOD               HOMING_POSITIVE_SWITCH
