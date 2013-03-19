#include "varext.h"
#include "def.h"
#include "sine_lookup.h"
#include "dc_motor_config.h"

void FOC_ClarkeAndPark()
{
	//============== clarke transformation ====================
			iAlpha =  iPhase1;
			iBeta  =  (iPhase1 + 2*iPhase2);   // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
			iBeta *=  37838;
			iBeta /=  65536;
			iBeta  =  -iBeta;

			//================== angle = arctg(iBeta/iAlpha) ===========
			iAngleCurrent = arctg1(iAlpha,iBeta);

			if(iStep1 == 0)
			{
				iAngleCurrent        = 0;
				iVectorCurrent       = 0;
			}

			 //============= park transform ==qq==========================
			theta = iAngleRotor/16;
			theta &= 0xFF;
			sinx  = sine_table[theta];      // sine( theta );
			theta = (64 - theta);  		    // 90-theta
			theta &= 0xFF;
			cosx  = sine_table[theta];      // values from 0 to +/- 16384
			iId = (((iAlpha * cosx )  /16384) + ((iBeta * sinx ) /16384));
			iIq = (((iBeta  * cosx )  /16384) - ((iAlpha * sinx ) /16384));

			iId *= 4;
			iIq *= 4;


			iIdPeriod += iId;
			iIqPeriod += iIq;
			iCountDivFactor++;

			if(cTriggerPeriod & 0x01  || iCountDivFactor > 18000)  // timeout 1 second
			{
				cTriggerPeriod &= 0x01^0xFF;
				iIqPeriod2 = iIqPeriod/iCountDivFactor;
				iIdPeriod2 = iIdPeriod/iCountDivFactor;
				iIqPeriod  = 0;
				iIdPeriod  = 0;
				iCountDivFactor=0;
			}

			if(iStep1==0)
			{
				VsdRef1 		= 0;
				VsqRef1 		= 0;
				iFieldDiffSum   = 0;
				iTorqueDiffSum  = 0;
			}
}// end FOC_ClarkeAndPark



void FOC_FilterDiffValue()
{
int iTemp1;

	iFieldDiffSum -= iFieldDiff2;
	iFieldDiffSum += iFieldDiff1;
	iFieldDiff2    = iFieldDiffSum/4;

	//------------- calc hysteresis and set limit -----------------
	iTemp1 = iTorqueDiff1;
	if(iTemp1 < 0) iTemp1   = -iTemp1;
	if(iTemp1 < 20) iTemp1=0;		// hys

	if(iTemp1 > 80) iTemp1 = 80;	// limit
	if(iTorqueDiff1 < 0)iTorqueDiff1 = -iTemp1;
	else iTorqueDiff1 = iTemp1;

	iTorqueDiffSum -= iTorqueDiff2;
	iTorqueDiffSum += iTorqueDiff1;
	iTorqueDiff2    = iTorqueDiffSum/4;
}





void FOC_Integrator(){
int iTemp1;

	//---------------- field --------------------------


	iFieldIntegral += iFieldDiff2  /16;

	iFieldProp      = iFieldDiff2  /16;

	if(iFieldIntegral >  16000)  iFieldIntegral =  16000;
	if(iFieldIntegral < -16000)  iFieldIntegral = -16000;

	VsdRef1 = iFieldIntegral + iFieldProp;
	//-------------------------------------------------


	//-------------- torque ---------------------------
	iTemp1 = iTorqueDiff2;

    if(iActualSpeed > iSetLoopSpeed)
    	if(iTemp1 > 0) iTemp1=0;

    iTorqueIntegral += idiffSpeed2/64;

	iTorqueIntegral += iTemp1 / 16;
	iTorqueProp      = iTemp1 /  8;

	VsqRef1 = iTorqueIntegral + iTorqueProp;

	if(iTorqueIntegral >  iTorqueLimit)  iTorqueIntegral =  iTorqueLimit;
	if(iTorqueIntegral < -iTorqueLimit)  iTorqueIntegral = -iTorqueLimit;
	//-------------------------------------------------



	if(iStep1==0)
	{
		iTorqueLimit 	 = TORQUE_INTEGRATOR_MAX;
		iFieldIntegral   = 0;
		iTorqueIntegral  = 0;
	}
}


void FOC_InversPark()
{
#define defVsqRef1Max 180000

		if(VsqRef1 > 0)
		{
        if(VsqRef1 > defVsqRef1Max)VsqRef1 = defVsqRef1Max;         // Limit
		}

		if(VsqRef1 < 0)
		{
        if(VsqRef1 < -defVsqRef1Max)VsqRef1 = -defVsqRef1Max;         // Limit
		}



	VsdRef2 = VsdRef1/ 16;
	VsqRef2 = VsqRef1/ 16;

 //================= invers park transformation =====================
		VsaRef = (VsdRef2 * cosx)/16384   - (VsqRef2 * sinx)/16384;
		VsbRef = (VsdRef2 * sinx)/16384   + (VsqRef2 * cosx)/16384;
		iAngleInvPark  = arctg1(VsaRef,VsbRef);           			// from 0 - 4095
}



void 	CalcCurrentValues()
{
int iTemp;
	//== cc ====== current low pass filter =============================
			// 66mV/A    16384 = 4.096Volt   2,5V = 16384/4,096 * 2,5 = 10000  16384/4096 * 66 = 264 bits/Ampere

			iPhase1Sum -= iPhase1;
			iPhase1Sum += a1;
			iPhase1     = iPhase1Sum/2;
			iTemp = iPhase1;	 if(iTemp < 0) iTemp = -iTemp;
			a1Square += (iTemp * iTemp);

			iPhase2Sum -= iPhase2;
			iPhase2Sum += a2;
			iPhase2     = iPhase2Sum/2;
			iTemp = iPhase2;	 if(iTemp < 0) iTemp = -iTemp;
			a2Square += (iTemp * iTemp);

			iCountRMS++;


			//=================== RMS =====================
			if(iCountRMS > 18000) iTriggerRMS = 5; // 100.000  / 55,556  =  18000     timeout about 100msec

			if(cTriggerPeriod & 0x02)
			{
				 cTriggerPeriod &= 0x02^0xFF;
				 iTriggerRMS++;
			}

			if(iTriggerRMS > 1)
				if(iCountRMS)
				{
					a1SquareMean = a1Square/iCountRMS;
					a2SquareMean = a2Square/iCountRMS;
					a1Square  = 0;
					a2Square  = 0;
					iCountRMS = 0;
					if(iTriggerRMS >= 5) iTriggerRMS=0;
					else iTriggerRMS--;
				}

			if(iVectorCurrent > iParRMS_PwmOff)   iStep1=30;  // Motor stop
			if(iVectorCurrent > iParRMS_PwmOff)   iStep1=30;  // Motor stop

			if(iPhase1 > ia1RMSMax)   // save last max value
				ia1RMSMax = iPhase1;
}



void SetParameterValue()
{
	iParRpmMotorMax 	=	iMotPar[0];
	iParRpmUmotMax		=	iMotPar[1];
	iParUmotStart		=	iMotPar[2];
	iParUmotBoost		=	iMotPar[3];

	iParAngleUser		=	iMotPar[5];

	iParRMS_RampLimit  	=   iMotPar[10];
	iParRMS_PwmOff      =   iMotPar[11];

	iParHysteresisPercent	=	iMotPar[16];
	iParDiffSpeedMax		=	iMotPar[17];
	iParUmotIntegralLimit	=	iMotPar[18];
	iParPropGain			= 	iMotPar[19];
	iParIntegralGain		=  	iMotPar[20];
}

//=================== default value =====================
void InitParameter(){
	iMotPar[0]  = defParRpmMotorMax;
	iMotPar[1]  = defParRpmUmotMax;
	iMotPar[2]  = defParUmotStart;
	iMotPar[3]  = defParUmotBoost;

	iMotPar[5]  = defParAngleUser;

	iMotPar[7] = defParEncoderResolution;
	iMotPar[8] = defParEncoderZeroPointPlus;
	iMotPar[9] = defParEncoderZeroPointMinus;


	iMotPar[10] = defParRmsLimit;				// ramp control
	iMotPar[11] = defParRmsMaxPwmOff;

	iMotPar[16] = defParHysteresisPercent;
	iMotPar[17] = defParDiffSpeedMax;			// SpeedControl
	iMotPar[18] = defParUmotIntegralLimit;
	iMotPar[19] = defParPropGain;
	iMotPar[20] = defParIntegralGain;

	iMotPar[24] = defParRampAcc;
	iMotPar[25] = defParRampDec;
	iMotPar[26] = defParRampSmoothFactor;

	iMotPar[28] = defParPositionSpeedMax;
	iMotPar[29] = defParPositionSpeedMin;
}


void SaveValueToArray()
{
	iMotValue[0]  = iUmotProfile*65536 + ((iUmotIntegrator/256)& 0xFFFF);
	iMotValue[1]  = iUmotBoost/256;
	iMotValue[2]  = iUmotP;
	iMotValue[3]  = iUmotMotor;
	iMotValue[4]  = iMotHoldingTorque;
	iMotValue[5]  = iControlFOC  + (iStep1 * 256)  + ((iMotDirection & 0xFF)) *65536;

	iMotValue[6]  = (iSetInternSpeed & 0xFFFF0000) + iSetLoopSpeed;  //(iSetSpeedRamp/65536);
	iMotValue[7]  = iActualSpeed;
	iMotValue[8]  = idiffSpeed*65536 + (idiffSpeed2 & 0xFFFF);
	iMotValue[9]  = iPositionAbsolutNew;
	iMotValue[10] = iEncoderActualSpeed;
	iMotValue[11] = 0;

	iMotValue[12] = (iHallAngle *65536) + iEncoderAngle;
	iMotValue[13] = (iAngleRotor*65536) + iAnglePWM;
	iMotValue[14] = iAngleRotorDiffCalculated;
	iMotValue[15] = 0;
	iMotValue[16] = 0;
	iMotValue[17] = iTorqueLimit; // adc_b4; //VsdRef1;

 	iMotValue[18] = iIq;   //adc_a1; //iFieldSet;
	iMotValue[19] = iIqPeriod2;   //adc_a2; //iIdPeriod2;
	iMotValue[20] = VsqRef2;   //adc_a3; //iFieldDiff2;
	iMotValue[21] = iTorqueSet;   //adc_a4; //iTorqueSet;
	iMotValue[22] = iTorqueDiff2;  // adc_b1; //;
	iMotValue[23] = iTorqueIntegral;  //adc_b2; //iTorqueDiff2;

	iMotValue[24] = a1RMS;
	iMotValue[25] = a2RMS;
	iMotValue[26] = iVectorCurrent;
	iMotValue[27] = iVectorInvPark;
	iMotValue[28] = iTorqueProp;

	iMotValue[29] = (iHallPinState*256)  + (iEncoderPinState & 0xFF);
	iMotValue[30] = iEncoderPositionAbsolut;
	iMotValue[31] = iHallPositionAbsolut;
}



