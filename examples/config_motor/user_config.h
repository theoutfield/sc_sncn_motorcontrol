/**
 * @file user_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <refclk.h>

#include <motor_config_AMK_DT4.h>

/////////////////////////////////////////////
//////  MOTOR SENSORS CONFIGURATION
/////////////////////////////////////////////

// SENSOR USED FOR COMMUTATION (if applicable) [HALL_SENSOR, AMS_SENSOR, CONTELEC_SENSOR, BISS_SENSOR]
#define MOTOR_COMMUTATION_SENSOR   CONTELEC_SENSOR

// POSITION SENSOR RESOLUTION [ticks/mechanical_rotation]
#define POSITION_SENSOR_RESOLUTION 65530

// SENSOR USED FOR CONTROL FEEDBACK [HALL_SENSOR, QEI_SENSOR, BISS_SENSOR]
#define MOTOR_FEEDBACK_SENSOR      MOTOR_COMMUTATION_SENSOR

// TYPE OF INCREMENTAL ENCODER (if applicable) [QEI_WITH_INDEX, QEI_WITH_NO_INDEX]
#define QEI_SENSOR_INDEX_TYPE      QEI_WITH_INDEX

// TYPE OF SIGNAL FOR INCREMENTAL ENCODER (if applicable) [QEI_RS422_SIGNAL, QEI_TTL_SIGNAL]
#define QEI_SENSOR_SIGNAL_TYPE     QEI_RS422_SIGNAL

// RESOLUTION OF YOUR INCREMENTAL ENCODER (if applicable)
#define QEI_SENSOR_RESOLUTION      4000

// POLARITY OF YOUR HALL SENSOR (if applicable) [1,-1]
#define SENSOR_POLARITY            1


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

#define I_MAX           100     //maximum tolerable value of phase current (under abnormal conditions)
#define V_DC_MAX        60      //maximum tolerable value of dc-bus voltage (under abnormal conditions)
#define V_DC_MIN        10      //minimum tolerable value of dc-bus voltave (under abnormal conditions)
#define TEMP_BOARD_MAX  100     //maximum tolerable value of board temperature (optional)


//////////////////////////////////////////////
//////  BRAKE CONFIGURATION
//////////////////////////////////////////////

#define DUTY_START_BRAKE    6000   // duty cycles for brake release (should be a number between 600 and 7000)
#define DUTY_MAINTAIN_BRAKE 1000   // duty cycles for keeping the brake released (should be a number between 700 and 7000)
#define PERIOD_START_BRAKE  1000   // period in which high voltage is applied for realising the brake [milli-seconds]
#define ENABLE_SHAKE_BRAKE     0

//////////////////////////////////////////////
//////  MOTOR COMMUTATION CONFIGURATION
//////////////////////////////////////////////

#define VDC             48

// COMMUTATION LOOP PERIOD (if applicable) [us]
#define COMMUTATION_LOOP_PERIOD     82
#define COMMUTATION_FRQ             24

//// COMMUTATION CW SPIN OFFSET (if applicable) [0:4095]
//#define COMMUTATION_OFFSET_CLK      0

// (OPTIONAL) MOTOR ANGLE IN EACH HALL STATE. IN CASE HALL SENSOR IS USED FIND THE
// FOLLOWING VALUES BY RUNNING OFFSET DETECTION FUNCTION, OR SET THEM ALL TO 0
#define HALL_STATE_1_ANGLE     0
#define HALL_STATE_2_ANGLE     0
#define HALL_STATE_3_ANGLE     0
#define HALL_STATE_4_ANGLE     0
#define HALL_STATE_5_ANGLE     0
#define HALL_STATE_6_ANGLE     0

//#define HALL_STATE_1_ANGLE     1754
//#define HALL_STATE_2_ANGLE     420
//#define HALL_STATE_3_ANGLE     1111
//#define HALL_STATE_4_ANGLE     3122
//#define HALL_STATE_5_ANGLE     2458
//#define HALL_STATE_6_ANGLE     3825


// MOTOR POLARITY [NORMAL_POLARITY, INVERTED_POLARITY]
#define MOTOR_POLARITY              NORMAL_POLARITY


///////////////////////////////////////////////
//////  MOTOR CONTROL CONFIGURATION
///////////////////////////////////////////////

// motor id (in case more than 1 motor is controlled)
#define MOTOR_ID 0

// PID FOR TORQUE CONTROL (if applicable) [will be divided by 10000]
#define TORQUE_Kp         40
#define TORQUE_Ki         40
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
#define TORQUE_CONTROL_LIMIT                    MAXIMUM_TORQUE

//Integrated Profiler
#define ENABLE_PROFILER                         0
#define MAX_ACCELERATION_PROFILER               1800000
#define MAX_SPEED_PROFILER                      1800000

/*
//PID parameters of the position PID controller
#define POSITION_Kp                             30000
#define POSITION_Ki                             10
#define POSITION_Kd                             0
#define POSITION_INTEGRAL_LIMIT                 400000 //in case of using non-linear position control,
                                                       //set "POSITION_INTEGRAL_LIMIT" to 1000
*/


//PID parameters of non-linear position controller. In case non-linear position controller is selected, these three
//constants "POSITION_Kp", "POSITION_Ki" and "POSITION_Kd" should be between 0 and 10^8. Inside the controller, these
//constants will be divided by 10^6. In other words, the precision in this mode will be 6 floating point digits.

/*
//-----  default values  -----
#define POSITION_Kp                             0
#define POSITION_Ki                             0
#define POSITION_Kd                             0

#define MAX_SPEED                               0    // prefered value 3000, maximum value 5000 [rpm]

#define POSITION_INTEGRAL_LIMIT                 1000 //in case of using non-linear position control,
                                                     //set "POSITION_INTEGRAL_LIMIT" to 1000

#define MOMENT_OF_INERTIA                       0    //set this variable only if it is known in [gram square centimiter]
                                                     //otherwise set as 0
*/

//-----  axis 4 of the robot (project FS)
#define POSITION_Kp                             76000
#define POSITION_Ki                             7700
#define POSITION_Kd                             320000

#define MAX_SPEED                               3000    // prefered value 3000, maximum value 5000 [rpm]

#define POSITION_INTEGRAL_LIMIT                 1000 //in case of using non-linear position control,
                                                     //set "POSITION_INTEGRAL_LIMIT" to 1000

#define MOMENT_OF_INERTIA                       0    //set this variable only if it is known in [gram square centimiter]
                                                     //otherwise set as 0
// COMMUTATION CW SPIN OFFSET (if applicable) [0:4095]
#define COMMUTATION_OFFSET_CLK      395



//PID parameters of the velocity PID controller
#define VELOCITY_Kp                             1500
#define VELOCITY_Ki                             4
#define VELOCITY_Kd                             0
#define VELOCITY_INTEGRAL_LIMIT                 6000


//Filter parameters
#define POSITION_FC             100
#define VELOCITY_FC             90

#define PID_GAIN                1000     //common gain multiplied in 3 pid constants at the same time.
                                         //default value is 1000 (which makes it ineffective)

//Number of ticks in a microsecond/frequency for IFM Tile
#define IFM_TILE_USEC   USEC_FAST//USEC_STD
