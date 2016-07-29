/**
 * @file motor_config_Nanotec_DB87S01.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              4       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT (MAXIMUM_TORQUE/10)      //motor torque constant multiplied by 100
#define RATED_CURRENT           6250    //rated phase current (RMS)
#define MAXIMUM_TORQUE          2128    //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             220     // rated power [W]
#define RATED_TORQUE            700     // rated motor torque [milli-Nm]
#define PEAK_SPEED              4500    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        180000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        350     // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
