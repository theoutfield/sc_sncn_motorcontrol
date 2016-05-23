/**
 * @file motor_config_Nanotec_DB42S03.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

/*
 *  Motor specific offst settings
 */

#define REGEN_V_HIGH          58
#define REGEN_V_LOW           38

#define REGEN_I_HIGH          60
#define REGEN_I_LOW           0

#define PERCENT_CHARGING_FACTOR_MAX  52
#define PERCENT_CHARGING_FACTOR_MIN  1


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
#define POLE_PAIRS             10


/////////////////////////////////////////////
//////  MOTOR PARAMETERS
////////////////////////////////////////////

// RATED POWER
#define RATED_POWER 140         // W

// RATED TORQUE
#define RATED_TORQUE 270        // mNm

// RATED CURRENT
#define RATED_CURRENT 5         // Amp

// PEAK SPEED
#define PEAK_SPEED    9000      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 552000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 720    // uH

// COMMUTATION PERIOD
#define COMMUTATION_PERIOD 66   // uS

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   57    // Nm at 1000 Amp

/*********************************************/



