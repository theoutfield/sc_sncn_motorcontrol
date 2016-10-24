/**
 * @file user_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>


//#include <motor_configs/motor_config_Nanotec_DB42C01.h>
//#include <motor_configs/motor_config_Nanotec_DB42C02.h>
//#include <motor_configs/motor_config_Nanotec_DB42C03.h>
//#include <motor_configs/motor_config_Nanotec_DB42L01.h>
//#include <motor_configs/motor_config_Nanotec_DB42M01.h>
//#include <motor_configs/motor_config_Nanotec_DB42M02.h>
//#include <motor_configs/motor_config_Nanotec_DB42M03.h>
//#include <motor_configs/motor_config_Nanotec_DB42S01.h>
//#include <motor_configs/motor_config_Nanotec_DB42S02.h>
//#include <motor_configs/motor_config_Nanotec_DB42S03.h>
//#include <motor_configs/motor_config_Nanotec_DB87S01.h>
//#include <motor_configs/motor_config_LDO_42BLS41.h>
//#include <motor_configs/motor_config_Moons_42BL30L2.h>
//#include <motor_config_Nanotec_DB59L024035-A.h>
//#include <motor_config_MABI_Hohlwellenservomotor_A5.h>
//#include <motor_config_MABI_A1.h>
//#include <motor_config_qmot_qbl5704.h>
//#include <motor_config_AMK_DT4.h>
#include <motor_config_AMK_DT3.h>
//#include <motor_config_AMK_DT2.h>
//#include <motor_config_AMK_DD7_28_10_T00.h>

//#include <motor_config.h>

/////////////////////////////////////////////
//////  MOTOR SENSORS CONFIGURATION
/////////////////////////////////////////////

// SENSOR USED FOR COMMUTATION (if applicable) [HALL_SENSOR]
#define MOTOR_COMMUTATION_SENSOR   CONTELEC_SENSOR

// SENSOR USED FOR CONTROL FEEDBACK [HALL_SENSOR, QEI_SENSOR, BISS_SENSOR]
#define MOTOR_FEEDBACK_SENSOR      MOTOR_COMMUTATION_SENSOR

// TYPE OF INCREMENTAL ENCODER (if applicable) [QEI_WITH_INDEX, QEI_WITH_NO_INDEX]
#define QEI_SENSOR_INDEX_TYPE       QEI_WITH_INDEX

// TYPE OF SIGNAL FOR INCREMENTAL ENCODER (if applicable) [QEI_RS422_SIGNAL, QEI_TTL_SIGNAL]
#define QEI_SENSOR_SIGNAL_TYPE      QEI_RS422_SIGNAL

// RESOLUTION OF YOUR INCREMENTAL ENCODER (if applicable)
#define QEI_SENSOR_RESOLUTION       4000

// POLARITY OF YOUR INCREMENTAL ENCODER (if applicable) [1, -1]
#define QEI_SENSOR_POLARITY         1

// POLARITY OF YOUR HALL SENSOR (if applicable) [1,-1]
#define HALL_POLARITY              1


//////////////////////////////////////////////
//////  RECUPERATION MODE PARAMETERS
//////////////////////////////////////////////

/*
 * WARNING: explosion danger. This mode shoule not be activated before evaluating battery behaviour.
 * */

// For not affecting higher controlling levels (such as position control),
// RECUPERATION should be set to 1, and REGEN_P_MAX should be set to a much higher value than the rated power
// (such as 50 kW),

#define RECUPERATION        1          // when RECUPERATION is 0, there will be no recuperation

#define BATTERY_E_MAX       80         // maximum energy status of battery
#define BATTERY_E_MIN       10         // minimum energy status of battery

#define REGEN_P_MAX         50000      // maximum regenerative power (in Watts)
#define REGEN_P_MIN         0          // minimum regenerative power (in Watts)

#define REGEN_SPEED_MAX     650
#define REGEN_SPEED_MIN     50         // minimum value of the speed which is considered in regenerative calculations


//////////////////////////////////////////////
//////  PROTECTION CONFIGURATION
//////////////////////////////////////////////

#define I_MAX           100      //maximum tolerable value of phase current (under abnormal conditions)
#define V_DC_MAX        60      //maximum tolerable value of dc-bus voltage (under abnormal conditions)
#define V_DC_MIN        10      //minimum tolerable value of dc-bus voltave (under abnormal conditions)
#define TEMP_BOARD_MAX  100     //maximum tolerable value of board temperature (optional)


//////////////////////////////////////////////
//////  BRAKE CONFIGURATION
//////////////////////////////////////////////
/*
//MABI PROJECT
#define DUTY_START_BRAKE    12000   // duty cycles for brake release (should be a number between 1500 and 13000)
#define DUTY_MAINTAIN_BRAKE 2000    // duty cycles for keeping the brake released (should be a number between 1500 and 13000)
*/

////FORESIGHT PROJECT
//#define DUTY_START_BRAKE    10000   // duty cycles for brake release (should be a number between 600 and 7000)
//#define DUTY_MAINTAIN_BRAKE  1500   // duty cycles for keeping the brake released (should be a number between 700 and 7000)

//FORESIGHT PROJECT
#define DUTY_START_BRAKE    6000   // duty cycles for brake release (should be a number between 600 and 7000)
#define DUTY_MAINTAIN_BRAKE 1000   // duty cycles for keeping the brake released (should be a number between 700 and 7000)


#define PERIOD_START_BRAKE  1000    // period in which high voltage is applied for realising the brake [milli-seconds]

//////////////////////////////////////////////
//////  MOTOR COMMUTATION CONFIGURATION
//////////////////////////////////////////////

#define VDC             48

// COMMUTATION LOOP PERIOD (if applicable) [us]
#define COMMUTATION_LOOP_PERIOD     66

// COMMUTATION CW SPIN OFFSET (if applicable) [0:4095]
#define COMMUTATION_OFFSET_CLK      1900

// MOTOR ANGLE IN EACH HALL STATE (should be configured in case HALL sensor is used)
#define HALL_STATE_1_ANGLE     0
#define HALL_STATE_2_ANGLE     0
#define HALL_STATE_3_ANGLE     0
#define HALL_STATE_4_ANGLE     0
#define HALL_STATE_5_ANGLE     0
#define HALL_STATE_6_ANGLE     0

// MOTOR POLARITY [NORMAL_POLARITY, INVERTED_POLARITY]
#define MOTOR_POLARITY              NORMAL_POLARITY


///////////////////////////////////////////////
//////  MOTOR CONTROL CONFIGURATION
///////////////////////////////////////////////

// motor id (in case more than 1 motor is controlled)
#define MOTOR_ID 0

// PID FOR TORQUE CONTROL (if applicable) [will be divided by 10000]
#define TORQUE_Kp         40 //7
#define TORQUE_Ki         40  //3
#define TORQUE_Kd         0

// (maximum) generated torque while finding offset value as a percentage of rated torque
#define PERCENT_OFFSET_TORQUE 80


/////////////////////////////////////////////////
//////  PROFILES AND LIMITS CONFIGURATION
/////////////////////////////////////////////////

// POLARITY OF THE MOVEMENT OF YOUR MOTOR [1,-1]
#define POLARITY           1

// PROFILER LIMITS
#define MAX_ACCELERATION        7000            // rpm/s
#define MAX_DECELERATION        7000            // rpm/s


/////////////////////////////////////////////////
//////  POSITION CONTROLLER
/////////////////////////////////////////////////
// POSITION CONTROL LOOP PERIOD [us]
#define CONTROL_LOOP_PERIOD     1000 //500

//Limits
#define MIN_POSITION_LIMIT                     -0x7fffffff
#define MAX_POSITION_LIMIT                      0x7fffffff
#define MAX_SPEED                               1500 //rpm
#define TORQUE_CONTROL_LIMIT                    3000

//Integrated Profiler
#define ENABLE_PROFILER                         1
#define MAX_ACCELERATION_PROFILER               1800000
#define MAX_SPEED_PROFILER                      1800000

/*
//PID parameters of the position PID controller
#define POSITION_Kp                             30000
#define POSITION_Ki                             10
#define POSITION_Kd                             0
*/

//PID parameters of non-linear position controller
#define POSITION_Kp                             989500
#define POSITION_Ki                             100100
#define POSITION_Kd                             4142100

#define POSITION_INTEGRAL_LIMIT                 400000
#define MOMENT_OF_INERTIA                       75      // [micro-kgm2]

//PID parameters of the velocity PID controller
#define VELOCITY_Kp                             100
#define VELOCITY_Ki                             0
#define VELOCITY_Kd                             60
#define VELOCITY_INTEGRAL_LIMIT                 0


//Filter parameters
#define POSITION_FC             100
#define VELOCITY_FC             90

//Number of ticks in a microsecond/frequency for IFM Tile
#define IFM_TILE_USEC   USEC_STD//USEC_FAST//
