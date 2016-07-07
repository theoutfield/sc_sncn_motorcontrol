/**
 * @file motor_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 *
 *   Example motor config file
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

/*
 * Set the Motor offset and position of the hall state sensors.
 * Unless for adding a new motor or fixing motor settings do not touch this.
 */


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

/*
// MABI AXIS_1 and AXIS_2
// RATED POWER
#define RATED_POWER 735         // W

// RATED TORQUE
#define RATED_TORQUE 540        // mNm

// RATED CURRENT
#define RATED_CURRENT 20         // Amp

// PEAK SPEED
#define PEAK_SPEED    3000      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 125000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 525    // uH

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   270    // Nm at 1000 Amp
*/


/*
// MABI AXIS_3 and AXIS_4
// RATED POWER
#define RATED_POWER 450         // W

// RATED TORQUE
#define RATED_TORQUE 143        // mNm

// RATED CURRENT
#define RATED_CURRENT 11         // Amp

// PEAK SPEED
#define PEAK_SPEED    5000      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 210000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 470    // uH

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   130    // Nm at 1000 Amp
*/

/*
// MABI AXIS_5 and AXIS_6
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

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   57    // Nm at 1000 Amp
*/

// Foresight

// RATED POWER
#define RATED_POWER 590         // W

// RATED TORQUE
#define RATED_TORQUE 1310        // mNm

// TORQUE CONSTANT
#define PERCENT_TORQUE_CONSTANT     10

// RATED CURRENT
#define RATED_CURRENT 4         // Amp

// PEAK SPEED
#define PEAK_SPEED    6800      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 128000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 80    // uH

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   100    // Nm at 1000 Amp


/*********************************************/

/////////////////////////////////////////////
//////  GENERAL MOTOR CONFIGURATION
////////////////////////////////////////////

// COMMUTATION PERIOD
#define COMMUTATION_PERIOD 66   // uS

// MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define MOTOR_TYPE  BLDC_MOTOR

// WINDING TYPE (if applicable) [STAR_WINDING, DELTA_WINDING]
#define BLDC_WINDING_TYPE   STAR_WINDING


