/**
 * @file user_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>

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
#include <motor_config_MABI_Hohlwellenservomotor_A5.h>
//#include <motor_config_qmot_qbl5704.h>
//#include <motor_config_AMK_DT3.h>

//#include <motor_config.h>

/////////////////////////////////////////////
//////  MOTOR SENSORS CONFIGURATION
///////////////////////////////////////////

// SENSOR USED FOR COMMUTATION (if applicable) [HALL_SENSOR]
#define MOTOR_COMMUTATION_SENSOR   BISS_SENSOR

// SENSOR USED FOR CONTROL FEEDBACK [HALL_SENSOR, QEI_SENSOR, BISS_SENSOR]
#define MOTOR_FEEDBACK_SENSOR      BISS_SENSOR

// TYPE OF INCREMENTAL ENCODER (if applicable) [QEI_WITH_INDEX, QEI_WITH_NO_INDEX]
#define QEI_SENSOR_INDEX_TYPE       QEI_WITH_INDEX

// TYPE OF SIGNAL FOR INCREMENTAL ENCODER (if applicable) [QEI_RS422_SIGNAL, QEI_TTL_SIGNAL]
#define QEI_SENSOR_SIGNAL_TYPE      QEI_RS422_SIGNAL

// RESOLUTION OF YOUR INCREMENTAL ENCODER (if applicable)
#define QEI_SENSOR_RESOLUTION       4000

// POLARITY OF YOUR INCREMENTAL ENCODER (if applicable) [1, -1]
#define QEI_SENSOR_POLARITY         1

//////////////////////////////////////////////
//////  PROTECTION CONFIGURATION
////////////////////////////////////////////
#define I_MAX           60      //maximum tolerable value of phase current (under abnormal conditions)
#define V_DC_MAX        50      //maximum tolerable value of dc-bus voltage (under abnormal conditions)
#define V_DC_MIN        10      //minimum tolerable value of dc-bus voltave (under abnormal conditions)
#define TEMP_BOARD_MAX  100     //maximum tolerable value of board temperature (optional)


//////////////////////////////////////////////
//////  MOTOR COMMUTATION CONFIGURATION
////////////////////////////////////////////
#define VDC             48
// COMMUTATION LOOP PERIOD (if applicable) [us]
#define COMMUTATION_LOOP_PERIOD     66

// COMMUTATION CW SPIN OFFSET (if applicable) [0:4095]
#define COMMUTATION_OFFSET_CLK      0 //3450 for qmot, 2435 mabi A2, 0 mabi A1

// COMMUTATION CCW SPIN OFFSET (if applicable) [0:4095]
#define COMMUTATION_OFFSET_CCLK     0


// MOTOR POLARITY [NORMAL_POLARITY, INVERTED_POLARITY]
#define MOTOR_POLARITY              INVERTED_POLARITY

///////////////////////////////////////////////
//////  MOTOR CONTROL CONFIGURATION
/////////////////////////////////////////////

// CONTROL LOOP PERIOD [us]
#define CONTROL_LOOP_PERIOD     1000

// PID FOR POSITION CONTROL (if applicable) [will be divided by 10000]
#define POSITION_Kp       100
#define POSITION_Ki       1
#define POSITION_Kd       0

// PID FOR VELOCITY CONTROL (if applicable) [will be divided by 10000]
#define VELOCITY_Kp       667
#define VELOCITY_Ki       200
#define VELOCITY_Kd       0

// PID FOR TORQUE CONTROL (if applicable) [will be divided by 10000]
#define TORQUE_Kp         1000
#define TORQUE_Ki         91
#define TORQUE_Kd         0

/////////////////////////////////////////////////
//////  PROFILES AND LIMITS CONFIGURATION
///////////////////////////////////////////////

// POLARITY OF THE MOVEMENT OF YOUR MOTOR [1,-1]
#define POLARITY           1

// DEFAULT PROFILER SETTINGS FOR PROFILE ETHERCAT DRIVE
#define PROFILE_VELOCITY        1000        // rpm
#define PROFILE_ACCELERATION    2000        // rpm/s
#define PROFILE_DECELERATION    2000        // rpm/s
#define PROFILE_TORQUE_SLOPE    400         // adc_ticks

// PROFILER LIMITIS
#define MAX_POSITION_LIMIT      0x7fffffff        // ticks (max range: 2^30, limited for safe operation)
#define MIN_POSITION_LIMIT      -0x7fffffff       // ticks (min range: -2^30, limited for safe operation)
#define MAX_VELOCITY            7000            // rpm
#define MAX_ACCELERATION        7000            // rpm/s
#define MAX_DECELERATION        7000            // rpm/s
#define MAX_CURRENT_VARIATION   800             // adc_ticks/s
#define MAX_CURRENT             800             // adc_ticks
