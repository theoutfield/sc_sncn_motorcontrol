/**
 * @file motor_config_Moons_42BL30L2.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        4       //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   100000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           1670    //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          1000    //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             30      // rated power [W]
#define RATED_TORQUE            72      // rated motor torque [milli-Nm]
#define PEAK_SPEED              5000    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        1340000 // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        1150    // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
