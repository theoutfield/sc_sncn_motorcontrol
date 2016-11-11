/**
 * @file motor_config.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 *
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              8       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT 5      //motor torque constant multiplied by 100
#define RATED_CURRENT           2330  //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          210   //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            70   //rated motor torque [milli-Nm].

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             50    // rated power [W]
#define PEAK_SPEED              6710    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        1030000     // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        572      // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
