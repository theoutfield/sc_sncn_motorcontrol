/**
 * @file motor_config_AMK_DT3.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        5       //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   170000  //Torque constant [micro-Nm/Amp-RMS]
#define MOTOR_RATED_CURRENT           4100    //rated phase current [milli-Amp-RMS]
#define MOTOR_MAXIMUM_TORQUE          5000    //maximum value of torque which can be produced by motor [milli-Nm]
#define MOTOR_RATED_TORQUE            620     // rated motor torque [milli-Nm]
#define MOTOR_MAX_SPEED               3000    // please update from the motor datasheet [rpm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             162     // rated power [W]
#define PEAK_SPEED              3200    // maximum motor speed [rpm]
#define MOTOR_PHASE_RESISTANCE  1270000 // motor phase resistance [micro-ohm]
#define MOTOR_PHASE_INDUCTANCE  1330    // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
