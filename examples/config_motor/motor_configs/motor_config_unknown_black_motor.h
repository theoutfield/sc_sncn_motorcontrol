/**
 * @file motor_config_unknown_black_motor.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define MOTOR_POLE_PAIRS        4       //number of motor pole-pairs
#define MOTOR_TORQUE_CONSTANT   130000  //Torque constant [micro-Nm/Amp-RMS]
#define RATED_CURRENT           20000   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          30000   //maximum value of torque which can be produced by motor [milli-Nm]
#define RATED_TORQUE            5000    //rated motor torque [milli-Nm]. CAUTION: CAN DAMAGE THE MOTOR OR INVERTER IF SET TOO HIGH

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             4000   // rated power [W]
#define PEAK_SPEED              5000   // maximum motor speed [rpm]
#define PHASE_RESISTANCE        6200   // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE          68   // motor phase inductance [micro-Henry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
