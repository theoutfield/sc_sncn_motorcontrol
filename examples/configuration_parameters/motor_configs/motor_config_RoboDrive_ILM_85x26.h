/**
 * @file motor_config_RobotDrive_ILM_85x26.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        10      //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   244000  //Torque constant [micro-Nm/Amp-RMS]
#define MOTOR_RATED_CURRENT           11000    //rated phase current [milli-Amp-RMS]
#define MOTOR_MAXIMUM_TORQUE          8300     //maximum value of torque which can be produced by motor [milli-Nm]
#define MOTOR_RATED_TORQUE            2600     // rated motor torque [milli-Nm]
#define MOTOR_MAX_SPEED               1500    // please update from the motor datasheet [rpm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             410     // rated power [W]
#define PEAK_SPEED              1500    // maximum motor speed [rpm]
#define MOTOR_PHASE_RESISTANCE  323000  // motor phase resistance [micro-ohm]
#define MOTOR_PHASE_INDUCTANCE  920     // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
