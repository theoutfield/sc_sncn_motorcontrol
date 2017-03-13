/**
 * @file motor_config_AMK_DD-28-10-T00
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        5       //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   120000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           110000  //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          16000   //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            14100   //rated motor torque [milli-Nm].

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             4900    // rated power [W]
#define PEAK_SPEED              3900    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        600     // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        49      // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
