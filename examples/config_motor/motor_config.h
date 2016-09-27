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

#define POWER_FACTOR           0

/////////////////////////////////////////////
//////  MOTOR PARAMETERS
////////////////////////////////////////////

// uncomment, and complete the following part depending on your motor model.
// You can also use the provided motor config file if it exists for you model

/*
// motor model:

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS                      //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT         //motor torque constant multiplied by 100
#define RATED_CURRENT                   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE                  //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE                    // rated motor torque [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER                     // rated power [W]
#define PEAK_SPEED                      // maximum motor speed [rpm]
#define PHASE_RESISTANCE                // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE                // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
*/

// 411678 MAXON motor

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              7         //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT 5         //motor torque constant multiplied by 100
#define RATED_CURRENT           5470      //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          2890      //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            289       // rated motor torque [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER          100        // rated power [W]
#define PEAK_SPEED          4000        // maximum motor speed [rpm]
#define PHASE_RESISTANCE    152000      // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE    188000      // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
