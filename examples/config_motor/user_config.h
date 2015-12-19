/**
 * @file bldc_motor_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */
#pragma once

#include <internal_config.h>
#include <qei_service.h>
#include <hall_service.h>
#include <motorcontrol_service.h>



#define MOTOR_TYPE  BLDC_MOTOR

/**
 * Define Motor Specific Constants (found in motor specification sheet)
 * Mandatory constants to be set
 */
#define POLE_PAIRS                  4               // Number of pole pairs
#define MAX_NOMINAL_SPEED           4000            // rpm
#define MAX_NOMINAL_CURRENT         2               // A
#define MOTOR_TORQUE_CONSTANT       72              // mNm/A

/**
 * If you have any gears added, specify gear-ratio
 * and any additional encoders attached specify encoder resolution here (Mandatory)
 */
#define GEAR_RATIO                  1                    // if no gears are attached - set to gear ratio to 1
#define ENCODER_RESOLUTION          4000               //  Resolution of Incremental Encoder (4X decoding - quadrature mode)

/* Position Sensor Types (select your sensor type here)
 * (HALL/ QEI) */
#define SENSOR_USED                QEI_SENSOR

/* Define your Incremental Encoder type (QEI_INDEX/ QEI_WITH_NO_INDEX) */
#define QEI_INDEX_TYPE             QEI_WITH_INDEX

/* QEI signal type. This parameter takes effect just in case
 * your dc board has a configurable qei port. Otherwise, the
 * only signal type accepted is RS422
 */
#define QEI_SIGNAL_TYPE            QEI_RS422_SIGNAL

/* Polarity is used to keep all position sensors to count ticks in the same direction
 *  (QEI_POLARITY_NORMAL/QEI_POLARITY_INVERTED) */
#define QEI_SENSOR_POLARITY         QEI_POLARITY_NORMAL

/* Somanet IFM Internal Config:  Specifies the current sensor resolution per Ampere
 *  (DC300_RESOLUTION / DC100_RESOLUTION / OLD_DC300_RESOLUTION) */
#define IFM_RESOLUTION              DC100_RESOLUTION

/* Commutation offset (range 0-4095) (HALL sensor based commutation) */
#define COMMUTATION_OFFSET_CLK      910//683
#define COMMUTATION_OFFSET_CCLK     2460//2731

/* Motor Winding type (STAR_WINDING/DELTA_WINDING) */
#define BLDC_WINDING_TYPE          DELTA_WINDING

/* Changes direction of the motor drive  (1 /-1) */
#define POLARITY                    1

/* Commutation loop period in us.
 * Reduce when sine commutation loops are combined. 250MHz reference clock is considered!
 */
#define COMMUTATION_LOOP_PERIOD     60    //us

/* Profile defines (Mandatory for profile modes) */
#define MAX_PROFILE_VELOCITY        MAX_NOMINAL_SPEED
#define PROFILE_VELOCITY            1000                // rpm
#define MAX_ACCELERATION            4000                // rpm/s
#define PROFILE_ACCELERATION        2000                // rpm/s
#define PROFILE_DECELERATION        2000                // rpm/s
#define QUICK_STOP_DECELERATION     2000                // rpm/s
#define PROFILE_TORQUE_SLOPE        66                  // (desired torque_slope/torque_constant)  * IFM resolution


/* Control specific constants/variables */

/* Position Control (Mandatory if Position control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define POSITION_Kp_NUMERATOR       1000
#define POSITION_Ki_NUMERATOR       1
#define POSITION_Kd_NUMERATOR       0

#if(SENSOR_USED == HALL_SENSOR)
    #define MAX_POSITION_LIMIT      POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO*10       // ticks (max range: 2^30, limited for safe operation) qei/hall/any position sensor
    #define MIN_POSITION_LIMIT      -POLE_PAIRS*HALL_POSITION_INTERPOLATED_RANGE*GEAR_RATIO*10      // ticks (min range: -2^30, limited for safe operation) qei/hall/any position sensor
#endif
#if(SENSOR_USED == QEI_SENSOR)
    #define MAX_POSITION_LIMIT      GEAR_RATIO*ENCODER_RESOLUTION*10    // ticks (max range: 2^30, limited for safe operation)
    #define MIN_POSITION_LIMIT      -GEAR_RATIO*ENCODER_RESOLUTION*10   // ticks (min range: -2^30, limited for safe operation)
#endif

/* Torque Control (Mandatory if Torque control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define TORQUE_Kp_NUMERATOR         1000
#define TORQUE_Ki_NUMERATOR         91
#define TORQUE_Kd_NUMERATOR         0

/* Velocity Control (Mandatory if Velocity control used)
 * possible range of gains Kp/Ki/Kd: 1/2^30 to 2^30
 * Note: gains are calculated as NUMERATOR/DENOMINATOR to give ranges */
#define VELOCITY_Kp_NUMERATOR       667
#define VELOCITY_Ki_NUMERATOR       200
#define VELOCITY_Kd_NUMERATOR       0


#define VELOCITY_FILTER_SIZE        8   //default (could be changed upto 16)

/* Define Homing method (HOMING_POSITIVE_SWITCH/HOMING_NEGATIVE_SWITCH)
 * this specifies direction for the node to find the home switch */
#define HOMING_METHOD               HOMING_POSITIVE_SWITCH
