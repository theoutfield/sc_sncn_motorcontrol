#include <xs1.h>
#include <stdint.h>
#include <print.h>
#include <xscope.h>
#include "pwm_cli_inv.h"
#include "sine_lookup.h"
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "hall_client.h"
#include "sine_lookup.h"
#include "comm_sine.h"


#ifdef DC900
extern out port p_ifm_ext_d0;
extern out port p_ifm_ext_d1;
extern out port p_ifm_ext_d2;
extern out port p_ifm_ext_d3;
#endif

int iMotPar[32];
int iMotValue[32];
//--------- pp -------------------------------
int iParRpmMotorMax;
int iParDefSpeedMax;
int iParRPMreference;
int iParSpeedKneeUmot;
int iParAngleUser;

int iParHysteresisPercent;
int iParDiffSpeedMax;
int iParAngleFromRPM;
int iParUmotIntegralLimit;
int iParPropGain;
int iParIntegralGain;
int iParUmotSocket;
int iParUmotBoost;
int iParUmotStart;

int iParRMS_RampLimit;
int iParRMS_PwmOff;


int iUpdateFlag=0;
//=========== motor values ===================================
int iUmotSquare;
int iUmotLinear;
int iUmotIntegrator=0;
int iUmotP;
int iUmotMotor = 0;   // follows iUmotResult
int iPositionAbsolut=0;
int a1RMS,a2RMS,a3RMS;
int iSetLoopSpeed=0;
int iActualSpeed=0;
int idiffSpeed;
int idiffSpeed2;  // idiffSpeed with hyteresis
int iIdPeriod2=0;
int iIqPeriod2=0;
int iAngleDiffPeriod;
int iAngleCorrection;
int iStep1 =0;
int iMotHoldingTorque =   0;

int iParAngleCorrVal;
int iParAngleCorrMax;

int iPositionEncoder;
int iPinStateHall;
int iPinStateEncoder=0;


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
	iMotValue[0]  = iUmotSquare;
	iMotValue[1]  = iUmotLinear;
	iMotValue[2]  = iUmotIntegrator/256;
	iMotValue[3]  = iUmotP;
	iMotValue[4]  = iUmotMotor;
	iMotValue[5]  = iSetLoopSpeed;
	iMotValue[6]  = iActualSpeed;
	iMotValue[7]  = idiffSpeed;
	iMotValue[8]  = idiffSpeed2;
	iMotValue[9]  = iMotHoldingTorque;
	iMotValue[10] = iStep1;
	iMotValue[11] = iPositionAbsolut;
	iMotValue[12] = iPositionEncoder;
	iMotValue[13] = iPinStateHall;
	iMotValue[14] = iPinStateEncoder;
	iMotValue[15] = iIdPeriod2;
	iMotValue[16] = iIqPeriod2;
	iMotValue[17] = iAngleDiffPeriod;
	iMotValue[18] = iAngleCorrection;
	iMotValue[19] = 0;
	iMotValue[20] = a1RMS;
	iMotValue[21] = a2RMS;
	iMotValue[22] = a3RMS;
	iMotValue[23] = 0;
	iMotValue[24] = 0;
	iMotValue[25] = 0;
	iMotValue[26] = 0;
	iMotValue[27] = 0;
	iMotValue[28] = 0;
	iMotValue[29] = 0;
	iMotValue[30] = 0;
	iMotValue[31] = 0;
}



#define defRampMax 8192		//ramp
#define defRampMin 32



void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl)
{
	unsigned cmd;
	unsigned pwm[3] = { 0, 0, 0 };
	int iTemp,iTemp1=0;
	int iTemp2,iTemp3,iTemp4;
	int iCountx;
	unsigned char cFlag=0;
	int iStepRamp=0;
	int iUmotBoost;
	int iUmotResult = 0;
	int iUmotLast   = 0;
	int iRampBlock  = 0;

	int iIndexPWM, iPosFromHallOld=0;
	int iAngleFromHall  = 0;

	int iAngleFromRpm   = 0;
	int iAnglePWM;

    int iAngleDiffSum;
    int iAngleDiff;
    int iIntegralGain;

    int iCountRMS  =0;
    int iCountAngle=0;
	int a1Square,a2Square,a3Square;
	int a1, a2, a3;
	int iAlpha,iBeta;
	int iAngleCur;
	int iPhase1=0;
	int iPhase2=0;
	int iPhase3=0;
	int iPhase1Sum=0;
	int iPhase2Sum=0;
	int iPhase3Sum=0;
	int iId;
	int iIdPeriod;
	int iIq;
	int iIqPeriod;

	int iCountDivFactor;

	int VsdRef,VsqRef;		// invers park
	int VsaRef,VsbRef;
	int iAngleInvPark,iVectorInvPark;

	int sinx,cosx;
	unsigned theta;  // angle

	unsigned uCurVector;

	int iRampAccValue=16;
	int iRampDecValue=16;
	int iRampIntegrator;

	int iSetValueSpeed=    0;
	int iSetInternSpeed=   0;
	int iSetSpeedRamp  =   0;

	char cTriggerPeriod=0;  // one complete hall period

    int iHysteresis;
	int iPwmOnOff = 1;
	int iSpeedValueNew	=  0;  // if speed from hall is a new value


    //-------------- init values --------------

	a1RMS = 0;
	a2RMS = 0;
	a3RMS = 0;
	iCountRMS=0;

	 InitParameter();
	 SetParameterValue();


	while (1)
	{
    cFlag |= 1;

#ifdef DC900
		p_ifm_ext_d0 <: cFlag;  // set to one
#endif

		//============= rotor position  from hall ===============================
		iPositionAbsolut = get_hall_absolute_pos(c_hall);
		iActualSpeed     = get_hall_speed(c_hall);
		iAngleFromHall   = get_hall_angle(c_hall);
		iPinStateHall    = get_hall_pinstate(c_hall);
		iPinStateEncoder = get_encoder_pinstate(c_hall);
		iPositionEncoder = get_encoder_position(c_hall);

		iSpeedValueNew = iActualSpeed & 0xFF000000;   				// extract info if SpeedValue is new
		iActualSpeed   &= 0x00FFFFFF;
		if(iActualSpeed & 0x00FF0000) iActualSpeed |= 0xFFFF0000;   // expand value if negativ


		if(iActualSpeed > 0)
		{
		if(iPosFromHallOld > 2048  && iAngleFromHall < 2048) cTriggerPeriod=0xFF;    //test
		}
		if(iActualSpeed < 0)
		{
		if(iPosFromHallOld < 2048  && iAngleFromHall > 2048) cTriggerPeriod=0x7F; 	//test
		}
		iPosFromHallOld = iAngleFromHall;




	//***************** Steps********************************************************

		 switch(iStep1)	//controller steps
		{
			case 0: iPwmOnOff = 0;
					if(iSetInternSpeed > 0){iUmotBoost = 250 * 32; iStep1= 1; }
					if(iSetInternSpeed < 0){iUmotBoost = 250 * 32; iStep1=21; }
					iIntegralGain = 0;
					iUmotResult   = 0;
					iStepRamp     = 1;
					iSetSpeedRamp = 0;
					iAngleCorrection = 0;
					break;

			case 1:  iStepRamp       = 1;
					 iPwmOnOff		 = 1;
					 iRampIntegrator = 0;
					 iUmotMotor = iUmotResult;
					 iStep1++;
					 break;

			case 2:  iUmotMotor = iUmotResult;
					 iStep1++; break;
			case 3:  iStep1++; break;
			case 4:  iStep1++; break;

			case 5:  if(iUmotBoost > 0)iUmotBoost--;
					 if(iUmotBoost > 0)iUmotBoost--;
					 if((iSetSpeedRamp + iRampIntegrator) >= iSetInternSpeed){
					 if(iRampAccValue > defRampMin) iRampAccValue-=8;
					 }
					 if(iSetSpeedRamp == iSetInternSpeed) iStep1=10;
					 if(iSetSpeedRamp  > iSetInternSpeed) iStep1=21;
					 break;

			case 10: iRampAccValue = defRampMin;
					 iRampDecValue = defRampMin;
					 iStepRamp     = 1;
					 if(iSetSpeedRamp  < iSetInternSpeed) iStep1=1;   // acc
					 if(iSetSpeedRamp  > iSetInternSpeed) iStep1=21;  // dec
					 if(iSetInternSpeed==0)iStep1++;
					 iCountx =0;
					 break;

			case 11: if(iCountx++ > 20000) iStep1=0; break;

			case 15: if(iSetSpeedRamp==0)iStep1=0;     // motor is stopping
					 break;

			//---------------------------------------------------

			case 21:  iStepRamp       = 1;
					  iPwmOnOff		  = 1;
					  iRampIntegrator = 0;
					  iUmotMotor = iUmotResult;
					  iRampAccValue =  defRampMin;
					  iStep1++;
					  break;
			case 22:  iStep1++; break;
			case 23:  iStep1++; break;
			case 24:  iStep1++; break;
			case 25:  if(iUmotBoost > 0)iUmotBoost--;
					  if(iUmotBoost > 0)iUmotBoost--;
					  if(iSetSpeedRamp == iSetInternSpeed) iStep1=10;
					  if(iSetSpeedRamp  < iSetInternSpeed) iStep1=1;
					  break;

			case 30:  iPwmOnOff		  = 0;			// error motor stop
					  iStep1++;
					  break;

			case 31:  iPwmOnOff		  = 0;
					  if(iSetLoopSpeed==0)iStep1=0;     // motor is stopping
					  break;

			default: printstr("error\n"); iStep1 = 0; break;
		}

		 //==========================================


		 iRampBlock = 0;
		 iTemp1 = iSetSpeedRamp;  	if(iTemp1 < 0) iTemp1 = -iTemp1;
		 iTemp2 = iSetInternSpeed;  if(iTemp2 < 0) iTemp2 = -iTemp2;
		 iTemp3 = iActualSpeed;     if(iTemp3 < 0) iTemp3 = -iTemp3;

		 if(iTemp1 < iTemp2)  // acceleration +/-
		 {
		 iTemp4 = iTemp1/65536 - iTemp3;
		     // if((iTemp4 - iTemp3) > 200) iRampBlock = 1;
		 }



		  //============================== ramp calculation ============
		 if(iSetSpeedRamp < iSetInternSpeed)  // acc == limit +
		 {
			 iRampDecValue = defRampMin;

			 if(iRampBlock==0)
			 iSetSpeedRamp += iRampAccValue;



			 if(iStepRamp==1)
			 {
				 if(iRampAccValue >= defRampMax)
				 { iStepRamp++; iRampAccValue = defRampMax;}
				 else
				 {
					 iRampIntegrator += iRampAccValue;
					 iRampAccValue += 8;
				 }
			 }
			 if(iSetSpeedRamp > iSetInternSpeed)iSetSpeedRamp = iSetInternSpeed;
		 }

		 if(iSetSpeedRamp > iSetInternSpeed)
		 {
			 iRampAccValue = defRampMin;
			 if(iRampBlock==0)
			 iSetSpeedRamp -= iRampDecValue;

			 iRampDecValue += 8;
			 if(iRampDecValue >= defRampMax)iRampDecValue = defRampMax;
			 if(iSetSpeedRamp < iSetInternSpeed)iSetSpeedRamp = iSetInternSpeed;
		 }

		 iSetLoopSpeed = iSetSpeedRamp/65536;


	 //== cc ====== current low pass filter =============================
	 // 66mV/A    16384 = 4.096Volt   2,5V = 16384/4,096 * 2,5 = 10000  16384/4096 * 66 = 264 bits/Ampere
	 iPhase1Sum -= iPhase1;
	 iPhase1Sum += a1;
	 iPhase1     = iPhase1Sum/2;

	 iPhase2Sum -= iPhase2;
	 iPhase2Sum += a2;
	 iPhase2     = iPhase2Sum/2;

	 iPhase3Sum -= iPhase3;
	 iPhase3Sum += a3;
	 iPhase3     = iPhase3Sum/2;

	 a1Square += (iPhase1 * iPhase1);
	 a2Square += (iPhase2 * iPhase2);
	 a3Square += (iPhase3 * iPhase3);

	 iCountRMS++;

	 iTemp = a1;
	 if(iTemp < 0) iTemp = -iTemp;
	 if(iTemp > 3500) iStep1=30;  			// Motor stop

	 if(a1RMS > iParRMS_PwmOff) iStep1=30;  // Motor stop
	 if(a2RMS > iParRMS_PwmOff) iStep1=30;  // Motor stop
//	 if(a3RMS > iParRMS_PwmOff) iStep1=30;  // Motor stop


	 //=================== RMS =====================
	 if(cTriggerPeriod & 0x02  || iCountRMS > 18000)  // 100.000  / 55,556  =  µsec timeout about 100msec
	 {
		 cTriggerPeriod &= 0x02^0xFF;

		 if(iCountRMS){
		 a1RMS = a1Square/iCountRMS;
		 a2RMS = a2Square/iCountRMS;
		 a3RMS = a3Square/iCountRMS;
		 }

		 a1RMS = root_function(a1RMS);
		 a2RMS = root_function(a2RMS);
		 a3RMS = root_function(a3RMS);

		 a1Square  = 0;
		 a2Square  = 0;
		 a3Square  = 0;
		 iCountRMS = 0;
	 }



	 //============== clarke transformation ===================
	 iAlpha = iPhase1;
	 iBeta  = (iPhase1 + 2*iPhase2);   // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
	 iBeta *= 37838;
	 iBeta /= 65536;
	 iBeta = -iBeta;

	 //================== angle = arctg(iBeta/iAlpha) =========
	 iAngleCur  = arctg1(iAlpha,iBeta);
	 uCurVector = iAlpha * iAlpha + iBeta * iBeta;

	 //============= park transform ==qq=========================
	theta = iAngleFromHall/16;
	theta &= 0xFF;
	sinx  = sine_table[theta];      // sine( theta );
	theta = (64 - theta);  		    // 90-theta
	theta &= 0xFF;
	cosx  = sine_table[theta];      // values from 0 to +/- 16384
	iId = (((iAlpha * cosx ) /16384) + ((iBeta * sinx ) /16384));
	iIq = (((iBeta * cosx ) /16384) - ((iAlpha * sinx ) /16384));

	iIdPeriod += iId;
	iIqPeriod += iIq;
	iCountDivFactor++;

	 if(cTriggerPeriod & 0x01  || iCountDivFactor > 18000)  // timeout 100 msec
	 {
		  cTriggerPeriod &= 0x01^0xFF;
		  iIqPeriod2 = iIqPeriod/iCountDivFactor;
		  iIdPeriod2 = iIdPeriod/iCountDivFactor;
		  iIqPeriod  = 0;
		  iIdPeriod  = 0;
		  iCountDivFactor=0;
	 }

	 //================ field and torque control =================

	 		 VsdRef = iId - 20;  //Period2;  // field

	 		 VsqRef = iIq;      //Period2;   // torque

	 //================= invers park transformation ==============

	 		VsaRef = VsdRef * cosx/16384   - VsqRef * sinx/16384;

	 		VsbRef = VsdRef * sinx/16384   + VsqRef * cosx/16384;

	 		iAngleInvPark  = arctg1(VsaRef,VsbRef);
	 		iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
	 		iVectorInvPark = root_function(iVectorInvPark);


    //**************************************************************
	//===dd============= speed-control =============================

	if(iSpeedValueNew)
	{
	idiffSpeed  = iSetLoopSpeed - iActualSpeed;
	idiffSpeed2 = idiffSpeed;

	iHysteresis  = iSetLoopSpeed;
	if(iHysteresis < 0) iHysteresis = -iHysteresis; // absolut value
	iHysteresis *= iParHysteresisPercent;
	iHysteresis /= 100;

		if(idiffSpeed > 0)
		{
		if(idiffSpeed < iHysteresis)  idiffSpeed2=0;  					   // Hysteresis
		if(idiffSpeed2 > iParDiffSpeedMax) idiffSpeed2 = iParDiffSpeedMax; // Limit
		}

		if(idiffSpeed < 0)
		{
		if(idiffSpeed > -iHysteresis)  idiffSpeed2=0;
		if(idiffSpeed2 < -iParDiffSpeedMax) idiffSpeed2 = -iParDiffSpeedMax;
		}

	if(iSetLoopSpeed < 0) idiffSpeed2 = -idiffSpeed2;  // invert if speed negativ
	if(iIntegralGain < (iParIntegralGain * 32)) iIntegralGain++;

	iUmotIntegrator += idiffSpeed2 * iIntegralGain / 256;
	iUmotP           = idiffSpeed2 * iParPropGain  /  32;

	//-------------- set limits -----------------------------------
	if(iUmotIntegrator > 0)
	if(iUmotIntegrator > (iParUmotIntegralLimit * 256))  iUmotIntegrator = (iParUmotIntegralLimit * 256);

	if(iUmotIntegrator < 0)
	if(iUmotIntegrator < -(iParUmotIntegralLimit * 256)) iUmotIntegrator = -(iParUmotIntegralLimit * 256);

	}// end if iSpeedValueNew
	//********************************************************************************************
	// end close loop
	//********************************************************************************************


	 //==================== uu  === umot calculation =================
	 iTemp = iSetLoopSpeed;
	 if(iTemp < 0) iTemp = -iTemp;

	 if(iTemp > iParSpeedKneeUmot) iTemp = iParSpeedKneeUmot; // set limit
	 iUmotSquare  = iTemp * iTemp;
	 iUmotSquare /= iParSpeedKneeUmot;
	 iUmotSquare *= 4096;
	 iUmotSquare /= iParSpeedKneeUmot;
	 iUmotSquare += iParUmotSocket;

	 iUmotLinear = iTemp * 4096;
	 iUmotLinear /= iParSpeedKneeUmot;
	 iUmotResult = iUmotLinear;

	 if(iUmotSquare > iUmotLinear) iUmotResult = iUmotSquare;
	 if(iUmotResult > 4096)   iUmotResult 	= 4096;
	//----------------------------------------------------
	 iUmotResult += iUmotIntegrator/256;
	 iUmotResult += iUmotP/256;
	 iUmotResult += (iUmotBoost / 32);

	 if(iUmotResult > 4096)   iUmotResult 	= 4096;
	 if(iUmotResult < 0 )     iUmotResult 	= 0;

	iRampBlock = 0;
	if(a1RMS >= iParRMS_RampLimit)
	{
	if(iUmotResult > iUmotLast) iUmotResult = iUmotLast;
	iRampBlock = 1;
	}
   	if(iStep1 == 0)
   	{
   		iUmotResult = 0;
   		if(iMotHoldingTorque)
   		{
   		iUmotResult = iMotHoldingTorque;
   		}
   	}

   	if(iActualSpeed >= iParRpmMotorMax)  // Limit Max Speed
   	{
   	if(iUmotResult > iUmotLast) iUmotResult = iUmotLast;
   	}
	//------------ iUmotMotor follows iUmotResult -----------
	iUmotLast = iUmotResult;
	if(iUmotResult > iUmotMotor) iUmotMotor++;
	if(iUmotResult < iUmotMotor) iUmotMotor--;

	//====== aa angle correction ================================================
	iAngleFromRpm = iActualSpeed;
	if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;  // absolut value
	iAngleFromRpm *= iParAngleFromRPM;
	iAngleFromRpm /= 4000;

	 iAngleDiff     = iAngleCur - iAngleFromHall;
	 iAngleDiff    &= 0x0FFF;
	 iAngleDiffSum += iAngleDiff;
	 iCountAngle++;

	 if(cTriggerPeriod & 0x04)
	 {
		 cTriggerPeriod &= 0x04^0xFF;
		 if(iCountAngle){
	   		 iAngleDiffPeriod = iAngleDiffSum/iCountAngle;
	    	 iAngleDiffSum    = 0;
	    	 iCountAngle      = 0;
		 }
	 }


	if (iSetLoopSpeed >= 0)
	{
		    if(iStep1 == 10)
		    if(a1RMS > 350)
		    {
		    if(iAngleDiffPeriod > (1024+16)) iAngleCorrection += iParAngleCorrVal;
		    if(iAngleDiffPeriod < (1024-16)) iAngleCorrection -= iParAngleCorrVal;
		    if(iAngleCorrection > iParAngleCorrMax )  iAngleCorrection =  iParAngleCorrMax;
		    if(iAngleCorrection < -iParAngleCorrMax)  iAngleCorrection = -iParAngleCorrMax;
		    }

			iAnglePWM = iAngleFromHall + iParAngleUser + iAngleFromRpm - iAngleCorrection/4 + 260;
	}


	if (iSetLoopSpeed <  0)
	{
					if(iStep1 == 10)
				    if(a1RMS > 350)
				    {
				    if(iAngleDiffPeriod > (4096-1024+16)) 	  iAngleCorrection -= iParAngleCorrVal;
				    if(iAngleDiffPeriod < (4096-1024-16)) 	  iAngleCorrection += iParAngleCorrVal;
				    if(iAngleCorrection > iParAngleCorrMax )  iAngleCorrection =  iParAngleCorrMax;
				    if(iAngleCorrection < -iParAngleCorrMax)  iAngleCorrection = -iParAngleCorrMax;
				    }
			iAnglePWM = iAngleFromHall + iParAngleUser - iAngleFromRpm  + iAngleCorrection/4  + 2500;
	}


	if(iStep1 == 0 )
		if(iMotHoldingTorque)
		{
			iAnglePWM = iAngleFromHall + iParAngleUser - 600;
		}

	iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff
	//=================================================================================================

	if(iStep1 == 0)
	{
	  iUmotIntegrator 	=0;
	  iUmotP			=0;
	  iIqPeriod2		=0;
	  iIdPeriod2		=0;
	  iAngleDiffPeriod	=0;
	}
/*
	 	 xscope_probe_data(0,a1);
	 	 xscope_probe_data(1,a2);
	 	 xscope_probe_data(2,iUmotMotor);
	 	 xscope_probe_data(3,iAngleFromHall);
	 	 xscope_probe_data(4,iAngleCur);
	   	 xscope_probe_data(5,iAngleDiffPeriod);
	   	 xscope_probe_data(6,iIq);
*/

	 xscope_probe_data(0,a1RMS);
	 xscope_probe_data(1,iActualSpeed);
	 xscope_probe_data(2,iSetLoopSpeed);
	 xscope_probe_data(3,iUmotIntegrator);
	 xscope_probe_data(4,iUmotMotor);
  	 xscope_probe_data(5,iTemp4);
  	 xscope_probe_data(6,iIqPeriod2);



	   	// pwm 13889 * 4 nsec = 55,556µsec  18Khz
		iIndexPWM = iAnglePWM >> 4;
		pwm[0] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM +85) & 0xff;
		pwm[1] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM + 86) & 0xff;
		pwm[2] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;


	#ifdef DC900
		if(sine_third[iIndexPWM] > 0) p_ifm_ext_d1 <: 1;
		else p_ifm_ext_d1 <: 0;
		p_ifm_ext_d0 <: 0; // yellow
		 {a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
		 a1 = -a1;
		 a2 = -a2;
		 a3 = -a1 -a2;
		p_ifm_ext_d2 <: 1;
	#endif


	 //======================== read current ============================
	 //  register ia ib
	#ifdef DC100
	 {a1, a2, a3}  = get_adc_vals_calibrated_int16_ltc1408( adc );
	 cFlag |= 0x02; testport <: cFlag;  // oszi green C4
	#endif


     if(iMotHoldingTorque == 0)
	 if(iPwmOnOff==0)
	 { pwm[0]=PWM_MAX_VALUE/2;   pwm[1]=PWM_MAX_VALUE/2;   pwm[2]=PWM_MAX_VALUE/2;  }


	  update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

	#ifdef DC900
	  p_ifm_ext_d2 <: 0;
	#endif

	SaveValueToArray();
	select
	{
		case c_commutation :> cmd:
		  p_ifm_ext_d3 <: 1;  // set to one

		  if     (cmd == 1)      { c_commutation :> iPwmOnOff;         }
		  else if(cmd == 2)      { c_commutation :> iMotHoldingTorque; }
		  else if(cmd == 4)      { c_commutation :> iSetValueSpeed;
		  	  	  	  	  	  	  if(iParDefSpeedMax > 0)
		  	  	  	  	  	  	  { iSetValueSpeed *= iParRPMreference;
		  	  	  	  	  	  	    iSetValueSpeed /= iParDefSpeedMax;
		  	  	  	  	  	  	  }
		  	  	  	  	  	  	  if(iSetValueSpeed > iParRpmMotorMax) iSetValueSpeed = iParRpmMotorMax;
		  	  	  	  	  	  	  iSetInternSpeed = iSetValueSpeed * 65536;
		  	  	  	  	  	     }

		  else if(cmd >= 8 && cmd < 32) { iTemp = (int) cmd; c_commutation <: iMotValue[iTemp-8]; 	}

		  else if(cmd >= 32 && cmd < 64) { iTemp = (int) cmd; c_commutation :> iTemp1;   iMotPar[iTemp-32] = iTemp1; iUpdateFlag=1;}
		  else if(cmd >= 64 && cmd < 96) { iTemp = (int) cmd; c_commutation <: iMotPar[iTemp-64]; 	}

		  break;
		default:
		  break;
		}// end select

		if(iUpdateFlag) { iUpdateFlag=0; SetParameterValue(); }

	    p_ifm_ext_d3 <: 0;

	}// end while(1)
}// end function






void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = { PWM_MAX_VALUE/2, PWM_MAX_VALUE/2, PWM_MAX_VALUE/2 };  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  do_adc_calibration_ad7949(c_adc);
	  comm_sine(c_adc, c_commutation, c_hall, c_pwm_ctrl);
}

