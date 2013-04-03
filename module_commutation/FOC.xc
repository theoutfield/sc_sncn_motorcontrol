#include "varext.h"
#include "def.h"
#include "sine_lookup.h"
#include "dc_motor_config.h"

unsigned root_function(unsigned uSquareValue);

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

	//		iId *= defCurrentFactor;   // 1 or 4
	//		iIq *= defCurrentFactor;


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

	if(iStep1==0)
		{
			iFieldDiffSum   = 0;
			iTorqueDiffSum  = 0;
		}
}





void FOC_Integrator(){
int iTemp1;
#define defREFERENZFACTOR  512

	//---------------- field --------------------------


	iFieldIntegrator += iFieldDiff2 /4;

	iFieldProp      = iFieldDiff2;

	if(iFieldIntegrator >  16000)  iFieldIntegrator =  16000;
	if(iFieldIntegrator < -16000)  iFieldIntegrator = -16000;

	iFieldReferenz = (iFieldIntegrator + iFieldProp) /defREFERENZFACTOR;
	//-------------------------------------------------------------------


	//-------------- torque ---------------------------------------------
	iTemp1 = iTorqueDiff2;


/*
	if(iSpeedIntegrator > 0)
	{
    if(iActualSpeed > iSpeedIntegrator/65536)
    	if(iTemp1 > 0) iTemp1=0;
    if(iSpeedValueIsNew) iTorqueUmotIntegrator += idiffSpeed2;
	}
    if(iSpeedIntegrator < 0)
    {
       if(iActualSpeed < iSpeedIntegrator/65536)
    	    	if(iTemp1 < 0) iTemp1=0;
       if(iSpeedValueIsNew) iTorqueUmotIntegrator += idiffSpeed2;
    }
*/


	iTorqueUmotIntegrator += iTemp1;
	iTorqueUmotIntegrator += idiffSpeed2;

	iTorqueProp            = iTemp1 * 4;

	if(iTorqueUmotIntegrator >  iTorqueLimit)  iTorqueUmotIntegrator =  iTorqueLimit;
	if(iTorqueUmotIntegrator < -iTorqueLimit)  iTorqueUmotIntegrator = -iTorqueLimit;

	iTorqueReferenz = (iTorqueUmotIntegrator + iTorqueProp) / defREFERENZFACTOR;
	//-------------------------------------------------

	if(iStep1==0)
	{
		iFieldIntegrator   		= 0;
		iTorqueUmotIntegrator   = 0;
	}
}


void FOC_InversPark()
{
 //================= invers park transformation =====================
		VsaRef = (iFieldReferenz * cosx)/16384   - (iTorqueReferenz * sinx)/16384;
		VsbRef = (iFieldReferenz * sinx)/16384   + (iTorqueReferenz * cosx)/16384;
		iAngleInvPark  = arctg1(VsaRef,VsbRef);           			// from 0 - 4095

		iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
		iVectorInvPark = root_function(iVectorInvPark);
}



//=======================================================================================
//
//=======================================================================================
//=======================================================================================






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

			if(iPhase1 > ia1RMSMax)   // save last max value
				ia1RMSMax = iPhase1;

			iLoopCount++;
			iLoopCount &= 0x01;
			switch(iLoopCount)
			{
				case 0: if(a1SquareMean)
							a1RMS = root_function(a1SquareMean);
							a1SquareMean = 0;
						break;

				case 1: if(a2SquareMean)
							a2RMS = root_function(a2SquareMean);
							a2SquareMean = 0;
						break;
			}




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
	iMotValue[5]  = iControlActual  + (iStep1 * 256)  + ((iMotDirection & 0xFF)) *65536;

	iMotValue[6]  = iSpeedSetUser/65536;
	iMotValue[7]  = iSpeedIntegrator/65536;
	iMotValue[8]  = iActualSpeed;
	iMotValue[9]  = idiffSpeed1;
	iMotValue[10] = idiffSpeed2;
	iMotValue[11] = iAngleDiffPeriod;

	iMotValue[12] = (iHallAngle *65536) + iEncoderAngle;
	iMotValue[13] = (iAngleRotor*65536) + iAnglePWM;
	iMotValue[14] = iAngleRotorDiffCalculated;
	iMotValue[15] = iDiffAngleRotor;
	iMotValue[16] = iAngleInvPark;
	iMotValue[17] = iTorqueLimit/defREFERENZFACTOR; // adc_b4; //VsdRef1;

 	iMotValue[18] = iTorqueSet;
	iMotValue[19] = iIq;
	iMotValue[20] = iIqPeriod2;
	iMotValue[21] = iTorqueDiff1;
	iMotValue[22] = iTorqueDiff2;
	iMotValue[23] = iTorqueReferenz;

	iMotValue[24] = a1RMS;
	iMotValue[25] = a2RMS;
	iMotValue[26] = iVectorCurrent;
	iMotValue[27] = iVectorInvPark;
	iMotValue[28] = iEncoderAbsolut;

	iMotValue[29] = (iHallPinState*256)  + (iEncoderPinState & 0xFF) + iEncoderNullReference *65536;
	iMotValue[30] = iEncoderPositionAbsolut;
	iMotValue[31] = iHallPositionAbsolut;
}


void SaveInfosToArray()
{
	iMotValue[0]  = hx1;
	iMotValue[1]  = hx2;
	iMotValue[2]  = hx3;
	iMotValue[3]  = hx4;
	iMotValue[4]  = hx5;
	iMotValue[5]  = hx6;
	iMotValue[6]  = hx7;

	iMotValue[7]  = hx8;
	iMotValue[8]  = hx9;
	iMotValue[9]  = hx10;
	iMotValue[10] = iEncoderActualSpeed;
	iMotValue[11] = iAngleRotor;
	iMotValue[12] = iHallAngle;
	iMotValue[13] = iEncoderAngle;

	iMotValue[14] = a1;
	iMotValue[15] = a2;
	iMotValue[16] = adc_a1;
	iMotValue[17] = adc_a2;
 	iMotValue[18] = adc_a3;
	iMotValue[19] = adc_a4;
	iMotValue[20] = adc_b1;

	iMotValue[21] = adc_b2;
	iMotValue[22] = adc_b3;
	iMotValue[23] = adc_b4;
	iMotValue[24] = a1RMS;
	iMotValue[25] = a2RMS;
	iMotValue[26] = iVectorCurrent;

	iMotValue[27] = iVectorInvPark;
	iMotValue[28] = iTorqueDiff2;
	iMotValue[29] = 0;
	iMotValue[30] = iEncoderPositionAbsolut;
	iMotValue[31] = iHallPositionAbsolut;
}

