/**
 * @file motor_config_Nanotec_DB42S03.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/


/*
 * Set the Motor offset and position of the hall state sensors.
 * Unless for adding a new motor or fixing motor settings do not touch this.
 */
#define MOTOR_OFFSET           0

#define MOTOR_HALL_STATE_1     0
#define MOTOR_HALL_STATE_2     0
#define MOTOR_HALL_STATE_3     0
#define MOTOR_HALL_STATE_4     0
#define MOTOR_HALL_STATE_5     0
#define MOTOR_HALL_STATE_6     0

#define POWER_FACTOR           0
#define POLE_PAIRS             5


/////////////////////////////////////////////
//////  MOTOR PARAMETERS
////////////////////////////////////////////

// RATED POWER
#define RATED_POWER 162         // W

// RATED TORQUE
#define RATED_TORQUE 620        // mNm

// TORQUE CONSTANT
#define PERCENT_TORQUE_CONSTANT     15

// RATED CURRENT
#define RATED_CURRENT 4         // Amp

// PEAK SPEED
#define PEAK_SPEED    3200      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 1270000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 1330    // uH

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   170    //maximum torque (Nm) at 1000 Amp

/*********************************************/

/////////////////////////////////////////////
//////  GENERAL MOTOR CONFIGURATION
////////////////////////////////////////////

// MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define MOTOR_TYPE  BLDC_MOTOR

// WINDING TYPE (if applicable) [STAR_WINDING, DELTA_WINDING]
#define BLDC_WINDING_TYPE   STAR_WINDING

