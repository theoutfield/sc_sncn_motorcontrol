/**
 * @file motor_config_Nanotec_DB42M02.h
 * @brief Motor Control config file (define your motor specifications here)
 * @author Synapticon GmbH <support@synapticon.com>
 */

/**************************************************
 *********      USER CONFIGURATION       **********
 **************************************************/

/////////////////////////////////////////////
//////  GENERAL MOTOR CONFIGURATION
////////////////////////////////////////////

// MOTOR TYPE [BLDC_MOTOR, BDC_MOTOR]
#define MOTOR_TYPE  BLDC_MOTOR

#define MOTOR_MAX_SPEED               3000    // please update from the motor datasheet [rpm]

// NUMBER OF POLE PAIRS (if applicable)
#define MOTOR_POLE_PAIRS  4

// WINDING TYPE (if applicable) [STAR_WINDING, DELTA_WINDING]
#define BLDC_WINDING_TYPE   DELTA_WINDING
