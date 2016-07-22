/**
 * @file motor_config_Nanotec_DB87S01.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

/////////////////////////////////////////////
//////  GENERAL MOTOR CONFIGURATION
////////////////////////////////////////////

// MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define MOTOR_TYPE  BLDC_MOTOR

// NUMBER OF POLE PAIRS (if applicable)
#define POLE_PAIRS  4

// RATED POWER
#define RATED_POWER 220         // W

// RATED TORQUE
#define RATED_TORQUE 700        // mNm

// RATED CURRENT
#define RATED_CURRENT 6         // Amp

// PEAK SPEED
#define PEAK_SPEED    4500      // rpm

// PHASE RESISTANCE
#define PHASE_RESISTANCE 180000 // uOhm

// PHASE INDUCTANCE
#define PHASE_INDUCTANCE 350    // uH

// MAXIMUM TORQUE
#define MAXIMUM_TORQUE   112    // Nm at 1000 Amp

// TORQUE CONSTANT
#define PERCENT_TORQUE_CONSTANT    (MAXIMUM_TORQUE/10)  //Nm at 100 Amp ???
