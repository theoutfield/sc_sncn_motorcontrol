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


// FAULHABER motor (150W)
// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        2       //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   30000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           4800   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          330   //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            111    //rated motor torque [milli-Nm]. CAUTION: CAN DAMAGE THE MOTOR OR INVERTER IF SET TOO HIGH

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             150   // rated power [W]
#define PEAK_SPEED              16000   // maximum motor speed [rpm]
#define PHASE_RESISTANCE        250000   // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        60   // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
