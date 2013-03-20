#include <xs1.h>
#include <stdint.h>
#include <xscope.h>
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "hall_client.h"
#include "comm_loop.h"
#include "var.h"
#include "def.h"

//#define DEBUG_commutation

static t_pwm_control pwm_ctrl;

#ifdef DEBUG_commutation
	#include <print.h>
extern out port p_ifm_ext_d1;
extern out port p_ifm_ext_d2;
extern out port p_ifm_shared_leds_wden;  // XS1_PORT_4B; /* BlueGreenRed_Green */
#endif

unsigned char cLeds;
int iPositionZero=0;

void comm_sine(chanend c_commutation, chanend c_value, chanend c_pwm_ctrl)
{
	int cmd;
	//-------------- init values --------------
	InitParameter();
	SetParameterValue();
	iControlFOC = 2;	iUmotMotor = 0; iMotHoldingTorque = 0;
	iStep1 = 0;
	 //================== pwmloop ========================
	while (1)
	{

		switch(iStep1)
		{
			case 0: iPwmOnOff 		 = 0; iIntegralGain    = 0;
					iUmotProfile     = 0; iUmotResult      = 0;
					iSetSpeedRamp    = 0; iMotDirection    = 0;
					if(iTorqueSet > 0){iMotDirection =   1; VsqRef1 = 4096;  iStep1 = 10;}   // ?? 4096  ?? 10
					if(iTorqueSet < 0){iMotDirection =  -1; VsqRef1 =-4096; iStep1 = 10;}
					break;

					//==========================================
			case 10: iPwmOnOff 		 = 1;
					 if(iTorqueSet  ==0)
						 iStep1++;
					 break;

			case 11: if(iActualSpeed == 0)iStep1=0;
					 break;
			//--------------- overcurrent --------------------------
			case 30:  iPwmOnOff		  = 0;			// error motor stop
					  iStep1++;
					  break;

			case 31:  iPwmOnOff		  =  0;
					  if(iControlFOC > 1)
					  {
						  if(iTorqueSet == 0 ) iStep1=0;
					  }
					  else
					  {
						  if(iSetLoopSpeed== 0)iStep1=0;     // motor is stopping
					  }
					  break;
			default:	iStep1 = 0;
						break;
		}// end iStep1


		if(iStep1==0)
		{
			iAngleCurrent        = 0;
			iVectorCurrent       = 0;
		}

		select
		{
			case c_value :> cmd:     // loop or user cmd token 40
				if(cmd == 40)
				{
					c_value :> cmd;   		// from user or other external loops
					iUmotMotor = cmd;      // also can be done but regulated angle
					c_value :> cmd;
					iIndexPWM = cmd;
				}
				break;
			default:
				break;

		}

		space_vector_pwm( iIndexPWM, iUmotMotor, iMotHoldingTorque, pwm_ctrl, c_pwm_ctrl, iPwmOnOff );

	}// end while(1)

}// end function






void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



void commutation(chanend  c_commutation,  chanend c_value, chanend c_pwm_ctrl, chanend sig)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  sig <: 1;
	  while(1)
	  {
		  unsigned cmd, found =0;
		  select
		  {
			case sig :> cmd:
				found = 1;
				break;
			default:
				break;
		  }
		  if(found == 1)
			  break;
	  }
	  //do_adc_calibration_ad7949(c_adc);
	  comm_sine(c_commutation, c_value, c_pwm_ctrl);
}
//===============================  utilities ===================================

void CalcSetInternSpeed(int iSpeedValue)
{
    iSetValueSpeed = iSpeedValue;
	if(iParDefSpeedMax > 0)
	{
		iSetValueSpeed *= iParRPMreference;
		iSetValueSpeed /= iParDefSpeedMax;
	}
	if(iSetValueSpeed > iParRpmMotorMax)	iSetValueSpeed = iParRpmMotorMax;
	iSetInternSpeed = iSetValueSpeed * 65536;
}




























void SetParameterValue()
{
	iParRpmMotorMax 	=	iMotPar[0];
	iParDefSpeedMax		=	iMotPar[1];
	iParRPMreference	=	iMotPar[2];
	iParAngleUser		=	iMotPar[3];
	iParAngleFromRPM	=	iMotPar[4];
	iParUmotBoost		=	iMotPar[5];
	iParUmotStart		=	iMotPar[6];
	iParSpeedKneeUmot	=	iMotPar[7];
	iParAngleCorrVal   	=   iMotPar[8];
	iParAngleCorrMax    =	iMotPar[9];
	iParRMS_RampLimit  	=   iMotPar[10];
	iParRMS_PwmOff      =   iMotPar[11];
	iMotPar[12] = 0;
	iMotPar[13] = 0;
	iMotPar[14] = 0;
	iMotPar[15] = 0;
	iParHysteresisPercent	=	iMotPar[16];
	iParDiffSpeedMax		=	iMotPar[17];
	iParUmotIntegralLimit	=	iMotPar[18];
	iParPropGain			= 	iMotPar[19];
	iParIntegralGain		=  	iMotPar[20];
	iMotPar[21] = 0;
	iMotPar[22] = 0;
	iMotPar[23] = 0;
	iMotPar[24] = 0;
	iMotPar[25] = 0;
	iMotPar[26] = 0;
	iMotPar[27] = 0;
	iMotPar[28] = 0;
	iMotPar[29] = 0;
	iMotPar[30] = 0;
	iMotPar[31] = 0;
}

//=================== default value =====================
void InitParameter(){
	iMotPar[0]  = defParRpmMotorMax;
	iMotPar[1]  = defParDefSpeedMax;
	iMotPar[2]  = defParRPMreference;

	iMotPar[3]  = defParAngleUser;
	iMotPar[4]  = defParAngleFromRPM;
	iMotPar[5]  = defParUmotBoost;
	iMotPar[6]  = defParUmotStart;
	iMotPar[7]  = defParSpeedKneeUmot;
	iMotPar[8]  = defParAngleCorrVal;
	iMotPar[9]  = defParAngleCorrMax;
	iMotPar[10] = defParRmsLimit;				// ramp control
	iMotPar[11] = defParRmsMaxPwmOff;
	iMotPar[12] = 0;
	iMotPar[13] = 0;
	iMotPar[14] = 0;
	iMotPar[15] = 0;
	iMotPar[16] = defParHysteresisPercent;
	iMotPar[17] = defParDiffSpeedMax;
	iMotPar[18] = defParUmotIntegralLimit;
	iMotPar[19] = defParPropGain;
	iMotPar[20] = defParIntegralGain;
	iMotPar[21] = 0;  	//
	iMotPar[22] = 0;  	//
	iMotPar[23] = 0;  	//
	iMotPar[24] = 0;  	//
	iMotPar[25] = 0;  	//
	iMotPar[26] = 0;  	//
	iMotPar[27] = 0;  	//
	iMotPar[28] = 0;  	//
	iMotPar[29] = 0;  	//
	iMotPar[30] = 0;  	//
	iMotPar[31] = 0;
}

void SaveValueToArray()
{
	iMotValue[0]  = iUmotProfile*65536 + (iUmotIntegrator/256);
	iMotValue[1]  = iUmotBoost/256;
	iMotValue[2]  = iUmotP;
	iMotValue[3]  = iUmotMotor;
	iMotValue[4]  = iMotHoldingTorque;
	iMotValue[5]  = iControlFOC  + (iStep1 * 256)  + ((iMotDirection & 0xFF) + iStepRamp*256) *65536;

	iMotValue[6]  = (iSetInternSpeed & 0xFFFF0000) + (iSetSpeedRamp/65536);
	iMotValue[7]  = iActualSpeed;
	iMotValue[8]  = idiffSpeed*65536 + (idiffSpeed2 & 0xFFFF);
	iMotValue[9]  = iPositionAbsolutNew;
	iMotValue[10] = iPositionDec;
	iMotValue[11] = iPulsCountAcc;


	iMotValue[12] = (iAngleFromHall*65536) + iAnglePWM;
	iMotValue[13] = iAngleDiffPeriod;
	iMotValue[14] = iRampAccValue;
	iMotValue[15] = 0;
	iMotValue[16] = VsqRef1;
	iMotValue[17] = VsdRef1;

 	iMotValue[18] = iFieldSet;
	iMotValue[19] = iIdPeriod2;
	iMotValue[20] = iFieldDiff2;
	iMotValue[21] = iTorqueSet;
	iMotValue[22] = iIqPeriod2;
	iMotValue[23] = iTorqueDiff2;

	iMotValue[24] = a1RMS;
	iMotValue[25] = a2RMS;
	iMotValue[26] = iVectorCurrent;
	iMotValue[27] = iVectorInvPark;
	iMotValue[28] = 0;

	iMotValue[29] = (iPinStateHall*256)  + (iPinStateEncoder & 0xFF);
	iMotValue[30] = iPositionEncoder;
	iMotValue[31] = iPositionAbsolut;
}



