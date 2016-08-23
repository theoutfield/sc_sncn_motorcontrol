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

/*
 * Should be set in case HALL sensor is used.
 */
#define HALL_STATE_1_ANGLE     0
#define HALL_STATE_2_ANGLE     0
#define HALL_STATE_3_ANGLE     0
#define HALL_STATE_4_ANGLE     0
#define HALL_STATE_5_ANGLE     0
#define HALL_STATE_6_ANGLE     0

#define POWER_FACTOR           0

/////////////////////////////////////////////
//////  MOTOR PARAMETERS
////////////////////////////////////////////

// uncomment, and complete the following part depending on your motor model.
// There are some examples in the following...

/*
// motor model:

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS                      //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT         //motor torque constant multiplied by 100
#define RATED_CURRENT                   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE                  //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER                     // rated power [W]
#define RATED_TORQUE                    // rated motor torque [milli-Nm]
#define PEAK_SPEED                      // maximum motor speed [rpm]
#define PHASE_RESISTANCE                // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE                // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define BLDC_WINDING_TYPE       STAR_WINDING    //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
*/


// motor model: DT4

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              5       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT 15      //motor torque constant multiplied by 100
#define RATED_CURRENT           8000    //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          2500    //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             300     // rated power [W]
#define RATED_TORQUE            1250    // rated motor torque [milli-Nm]
#define PEAK_SPEED              3700    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        490000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        580     // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]


/*
// motor model: MABI AXIS_1 and AXIS_2

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              ?       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT         //motor torque constant multiplied by 100
#define RATED_CURRENT           20000   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          ?       //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             735     // rated power [W]
#define RATED_TORQUE            540     // rated motor torque [milli-Nm]
#define PEAK_SPEED              3000    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        125000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        525     // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define BLDC_WINDING_TYPE       STAR_WINDING    //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
*/

/*
// motor model: MABI AXIS_3 and AXIS_4

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              ?       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT ?       //motor torque constant multiplied by 100
#define RATED_CURRENT           11000   //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          ?       //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             450     // rated power [W]
#define RATED_TORQUE            143     // rated motor torque [milli-Nm]
#define PEAK_SPEED              5000    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        210000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        470     // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define BLDC_WINDING_TYPE       STAR_WINDING    //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
*/

/*
// motor model: MABI AXIS_5 and AXIS_6

// IMPORTANT PARAMETERS (=> lead to mulfunction or damage if set wrong)
#define POLE_PAIRS              ?       //number of motor pole-pairs
#define PERCENT_TORQUE_CONSTANT ?       //motor torque constant multiplied by 100
#define RATED_CURRENT           5000    //rated phase current [milli-Amp-RMS]
#define MAXIMUM_TORQUE          ?       //maximum value of torque which can be produced by motor [milli-Nm]

// OTHER PARAMETERS (do not change if not having access to the following parameter values)
#define RATED_POWER             140     // rated power [W]
#define RATED_TORQUE            270     // rated motor torque [milli-Nm]
#define PEAK_SPEED              9000    // maximum motor speed [rpm]
#define PHASE_RESISTANCE        552000  // motor phase resistance [micro-ohm]
#define PHASE_INDUCTANCE        720     // motor phase inductance [micro-Hunnry]

// GENERAL PARAMETERS
#define MOTOR_TYPE              BLDC_MOTOR      //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define BLDC_WINDING_TYPE       STAR_WINDING    //MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
*/



