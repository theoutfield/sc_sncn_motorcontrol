/**
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-02-22 1850)
 * \
 */

#ifndef _DC_MOTOR_CONFIG__H_
#define _DC_MOTOR_CONFIG__H_
#pragma once

#define UMOT_MAX 4096
#define TORQUE_INTEGRATOR_MAX  UMOT_MAX*32

//#define defENCODER
#define defHALL

//=========================================================
//#define MOTOR_MAXON_EC45
#define MOTOR_MAXON_EC60

//==========================================================
#ifdef  MOTOR_MAXON_EC45

#define MOTOR_POWER 50
#define POLE_PAIRS	8
#define GEAR_RATIO	26
#define MAX_NOMINAL_SPEED  4000   // in 1/min
#define MAX_NOMINAL_CURRENT  5    // in A

#define defParRpmMotorMax		 6500
#define defParRpmUmotMax	 	11500
#define defParUmotBoost  		   60
#define defParUmotStart 		   70
#define defParAngleUser 		  640
//======== current limits ===============
#define defParRmsLimit			  750  // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff       2000  //
//============ speed_control ============
#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			    0
#define defParIntegralGain		    0
#define defCurrentFactor            4

#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64
//-------------------------------------
#define defParEncoderResolution       4000
#define defParEncoderZeroPointPlus     640
#define defParEncoderZeroPointMinus   1100
//-------------------------------------
#define defParRampAcc  			65536/2		 // change to RPM/sec
#define defParRampDec  			65536/2         //
#define defParRampSmoothFactor 		4     //

#define defParPositionSpeedMax    800   // RPM
#define defParPositionSpeedMin    100   // RPM
#endif
//=====================================================================

//==================================================================
#ifdef  MOTOR_MAXON_EC60

#define MOTOR_POWER 100
#define POLE_PAIRS	7
#define GEAR_RATIO	156
#define MAX_NOMINAL_SPEED  3700   // in 1/min
#define MAX_NOMINAL_CURRENT  5    // in A

#define defParRpmMotorMax		3700
#define defParRpmUmotMax	 	4000
#define defParUmotBoost  		 120
#define defParUmotStart 		 150
#define defParAngleUser 		 640
//======== current limits ===============
#define defParRmsLimit			750   // 66*4 = 264Bits/A
#define defParRmsMaxPwmOff      2000  //

//============ speed_control ============
#define defParEncoderResolution       4000
#define defParEncoderZeroPointPlus     640
#define defParEncoderZeroPointMinus   1100
//-------------------------------------

#define defParHysteresisPercent	    5
#define defParDiffSpeedMax		  150
#define defParUmotIntegralLimit	 2048
#define defParPropGain			    8
#define defParIntegralGain		    8
#define defCurrentFactor            1

#define defParTorquePropGain	   64
#define defParTorqueIntegralGain   64
//-------------------------------------
#define defParEncoderResolution 4000
#define defParEncoderZeroPoint   570
//-------------------------------------
#define defParRampAcc  65536/8		 //  change to RPM/sec
#define defParRampDec  65536/8
#define defParRampSmoothFactor 4

#define defParPositionSpeedMax   800   // RPM
#define defParPositionSpeedMin   100   // RPM
#endif
//=============================================================




#endif
