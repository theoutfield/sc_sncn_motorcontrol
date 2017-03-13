/**
 * @file motor_config_RobotDrive_ILM_50x08.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              10      //number of motor pole-pairs
#define TORQUE_CONSTANT         60000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           4800    //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          900     //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            270     // rated motor torque [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             140     // rated power [W]
#define PEAK_SPEED              9000    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        552000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        720     // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
