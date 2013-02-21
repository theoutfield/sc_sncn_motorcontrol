#include <xs1.h>
#include <stdint.h>
#include <xscope.h>
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "hall_client.h"
#include "comm_loop.h"

#define DEBUG_commutation

static t_pwm_control pwm_ctrl;

#ifdef DEBUG_commutation
	#include <print.h>

	extern out port p_ifm_ext_d2;
	extern out port p_ifm_shared_leds_wden;  // XS1_PORT_4B; /* BlueGreenRed_Green */
#endif

unsigned char cLeds;

int iDiffAngleHall;int iAngleXXX;int iUmotSquare;int iUmotLinear;int iRampAccValue=16;int iRampDecValue=16;

int iMotPar[32];int iMotValue[32];
//--------- pp -------------------------------
int iParRpmMotorMax;int iParDefSpeedMax;int iParRPMreference;int iParSpeedKneeUmot;int iParAngleUser;

int iParHysteresisPercent;int iParDiffSpeedMax;int iParAngleFromRPM;int iParUmotIntegralLimit;int iParPropGain;
int iParIntegralGain;int iParUmotSocket;int iParUmotBoost;int iParUmotStart;int iParRMS_RampLimit;int iParRMS_PwmOff;
int iUpdateFlag=0;

//=========== motor values ===================================
int iUmotProfile;int iUmotIntegrator=0;int iUmotP;int iUmotMotor = 0;   	// follows iUmotResult

int iTorqueDiff1;int iTorqueDiff2;int iTorqueDiffSum;int iTorqueSet=200;

int iFieldDiff1;int iFieldDiff2;int iFieldDiffSum;int iFieldSet=0;

int iPositionAbsolut=0;unsigned a1RMS,a2RMS,a3RMS; int ia1RMSMax=0;int iSetLoopSpeed=0;int iActualSpeed=0;int idiffSpeed;
int idiffSpeed2; /* idiffSpeed with hyteresis*/ int iIdPeriod2=0;int iIqPeriod2=0;int iAngleDiffPeriod;
int iPowerMotor = 0;
int iStep1 =0; int iMotHoldingTorque =   0;
int iParAngleCorrVal;int iParAngleCorrMax;

int iPositionEncoder;int iPinStateHall;int iPinStateEncoder=0;
int iRampIntegrator; int iSetValueSpeed	=  0;int iSetInternSpeed	=  0;int iSetInternSpeed2=  0;int iSetSpeedRamp  	=  0;
int iMotDirection  	=  0;int iControlFOC   	=  0;
//=======================================================
int iCountx;int iStepRamp=0;

int iUmotBoost  = 0;int iUmotResult = 0;int iUmotLast   = 0;int iRampBlock  = 0;
int iIndexPWM, iAngleFromHallOld=0;int iAngleFromHall  = 0;
int iAngleFromRpm   = 0;int iAnglePWM;int iAnglePWMFromHall;int iAnglePWMFromFOC;
int iAngleDiffSum;int iAngleDiff;int iIntegralGain;

int iCountRMS  =0;	int iTriggerRMS=0;	int iCountAngle=0;
unsigned a1Square=0,a2Square=0;unsigned a1SquareMean=0,a2SquareMean=0;
int a1=0, a2=0;int iPhase1		=0;int iPhase2		=0;int iPhase1Sum	=0;
int iPhase2Sum	=0;

int iAlpha,iBeta;int iAngleCurrent;int iId;int iIdPeriod;int iIq;int iIqPeriod;
int VsdRef1, VsqRef1;		// invers park
int VsdRef2, VsqRef2;		// invers park
int VsaRef, VsbRef;int iAngleInvPark;int iVectorInvPark;int sinx,cosx;unsigned theta;  // angle
unsigned iVectorCurrent;int iAngleDiffFOC;

#define defRampMax 8192		//ramp
#define defRampMin 64
void SpeedControl(); void CalcUmotForSpeed(); void CalcRampForSpeed(); void SaveValueToArray(); void SetParameterValue(); void InitParameter();

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue)
{
	unsigned cmd1;
	unsigned cmd2;
	int iTemp,iTemp1=0;
	unsigned char cFlag=0;

	int iLoopCount=0;
	int iCountDivFactor;

	char cTriggerPeriod=0;  // one complete hall period
	int iPwmOnOff 		= 1;
	int iSpeedValueNew	=  0;  // if speed from hall is a new value
	int iTorqueF0=0;
	//============================================
	int iPwmAddValue,iPwmIndexHigh;


    //-------------- init values --------------
   	iCountRMS=0;
	InitParameter();
	SetParameterValue();

	 //================== pwmloop ========================
	while (1)
	{
		#ifdef DEBUG_commutation
			cFlag |= 1;
			#ifdef DC900
	//			p_ifm_ext_d0 <: cFlag;  // set to one
			#endif
		#endif

		//============= rotor position  from hall ===============================
		iLoopCount++;
		iLoopCount &= 0x0F;
		switch(iLoopCount & 0x03)
		{
			case 0: iPositionAbsolut = get_hall_absolute_pos(c_hall);
					break;
			case 1: iPinStateHall    = get_hall_pinstate(c_hall);
					break;
			case 2: iPinStateEncoder = get_encoder_pinstate(c_hall);
					break;
			case 3: iPositionEncoder = get_encoder_position(c_hall);
					break;
		}
		iActualSpeed     =  get_hall_speed(c_hall);
		iAngleFromHall   = 	get_hall_angle(c_hall);
		iAngleFromHall  &=  0x0FFF;


		iSpeedValueNew   = iActualSpeed & 0xFF000000;   				// extract info if SpeedValue is new
		iActualSpeed    &= 0x00FFFFFF;
		if(iActualSpeed & 0x00FF0000)
			iActualSpeed |= 0xFFFF0000;   							// expand value if negativ


		if(iActualSpeed > 0)
		{
			if(iAngleFromHallOld > 2048  && iAngleFromHall < 2048)
			cTriggerPeriod = 0xFF;
		}
		if(iActualSpeed < 0)
		{
			if(iAngleFromHallOld < 2048  && iAngleFromHall > 2048)
			cTriggerPeriod = 0x7F;
		}


		if(iAngleFromHall != iAngleFromHallOld)
		{
			if(iAngleFromHall > iAngleFromHallOld)
				iDiffAngleHall = iAngleFromHall - iAngleFromHallOld;
			else
				iDiffAngleHall = (iAngleFromHall +4096) - iAngleFromHallOld;
		}

		iAngleFromHallOld = iAngleFromHall;


		//***************** steps ********************************************************


		switch(iStep1)
		{
			case 0: iPwmOnOff 		 = 0;
					iIntegralGain    = 0;
					iUmotProfile     = 0;
					iUmotResult      = 0;
					iStepRamp        = 1;
					iSetSpeedRamp    = 0;
					iAngleDiffFOC    = 1020;

					switch(iControlFOC)
					{
					case 0:
					case 1:
					if(iSetInternSpeed > 0){iMotDirection =  1;  VsqRef1=  1024; VsdRef1 = 512; iTorqueF0 =  500;  iStep1= 1; }
					if(iSetInternSpeed < 0){iMotDirection = -1;  VsqRef1= -1024; VsdRef1 = 512; iTorqueF0 = -500;  iStep1= 1; }
					break;

					case 2:
					if(iTorqueSet > 0){iMotDirection =   1; VsqRef1 = 4096;  iStep1 = 50;}
					if(iTorqueSet < 0){iMotDirection =  -1; VsqRef1 =-4096;  iStep1 = 50;}
					break;


					case 3:
					if(iSetInternSpeed > 0){iMotDirection =  1;  VsqRef1=  1024; VsdRef1 = 512; iTorqueF0 =  500;  iStep1= 80; }
					iPwmAddValue  = 65536;
					iPwmIndexHigh = 0;
					break;
					}// end switch iControlFOC


					break;

			case 1:  iUmotBoost = iParUmotBoost * 32;
			         iUmotMotor = iParUmotStart;
				     iStepRamp  = 1;
					 iPwmOnOff	= 1;
					 iRampIntegrator = 0;
					 iStep1++;
					 break;

			case 2:	 iStep1++; break;
			case 3:  iStep1++; break;
			case 4:  iStep1++; break;

			case 5:  if(iUmotBoost > 0)iUmotBoost--;

			//		 if((iSetSpeedRamp + iRampIntegrator) >= iSetInternSpeed){
			//		 if(iRampAccValue > defRampMin) iRampAccValue-=8;
			//		 }
					 if(iSetSpeedRamp == iSetInternSpeed) iStep1++;
					 break;

			case 6:  iStep1++; break;
			case 7:  iStep1++; break;
			case 8:  iStep1++; break;
			case 9:  iStep1++;
					 iStepRamp     =  1;
					 iRampAccValue =  defRampMin;
					 iRampDecValue =  defRampMin;
			         break;

			case 10: if(iSetInternSpeed == 0) iStep1++;
					 else
					 {
					   if(iSetInternSpeed > 0 && iSetSpeedRamp < 0)iStep1=20;
					   if(iSetInternSpeed < 0 && iSetSpeedRamp > 0)iStep1=20;
					   if(iStep1==10)
					   if(iSetSpeedRamp != iSetInternSpeed)   iStep1 = 5;
					 }
			         if(iControlFOC > 1) iStep1=50;
					 break;

			case 11: iCountx = 0;
			         iStep1++;
			         break;

			case 12: if(iSetSpeedRamp==0)  iStep1++;
			         if(iCountx++ > 30000) iStep1=0;
			         break;

			case 13: if(iCountx++ > 20000) iStep1++;
						break;
			case 14: iStep1++;
						break;
			case 15: iStep1=0;     							// motor is stopping
						break;

			//---------------------------------------------------
			case 20: iSetInternSpeed2 = iSetInternSpeed;
				     iSetInternSpeed = 0;
					 iCountx = 0;
                     iStep1++;
                     break;
			case 21: if(iSetSpeedRamp==0)  iStep1++;
			         break;
			case 22: if(iCountx++ > 9000)  iStep1++;  // wait 50msec
			         break;
			case 23: iSetInternSpeed = iSetInternSpeed2;
			         iStep1 = 0;
			         break;

			//--------------- overcurrent --------------------------
			case 30:  iPwmOnOff		  = 0;			// error motor stop
					  iStep1++;
					  break;

			case 31:  iPwmOnOff		  =  0;
			 	 	  if(iControlFOC > 1){
			 	 	  if(iTorqueSet == 0 ) iStep1=0;
			 	 	  }
			 	 	  else
			 	 	  {
					  if(iSetLoopSpeed== 0)iStep1=0;     // motor is stopping
			 	 	  }
			 	 	  break;

			//==========================================
			case 50: iPwmOnOff 		 = 1;
					 if(iTorqueSet == 0)iStep1 = 0;
					 if(iControlFOC ==0)iStep1 = 0;
				     break;


		   //===========================================
			case 80:  iUmotResult = iSetInternSpeed/65536;
			         if(iSetInternSpeed==0)iStep1=0;
				     if(iControlFOC ==0)iStep1 = 0;
				     break;


			default:
					#ifdef DEBUG_commutation
						printstr("error\n");
					#endif
					iStep1 = 0;
					break;
		}


		//===========================================================
		CalcRampForSpeed();
		//============================== ramp calculation ===========


		//== cc ====== current low pass filter =============================
		// 66mV/A    16384 = 4.096Volt   2,5V = 16384/4,096 * 2,5 = 10000  16384/4096 * 66 = 264 bits/Ampere
//c
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
		theta = iAngleFromHall/16;
		theta &= 0xFF;
		sinx  = sine_table[theta];      // sine( theta );
		theta = (64 - theta);  		    // 90-theta
		theta &= 0xFF;
		cosx  = sine_table[theta];      // values from 0 to +/- 16384
		iId = (((iAlpha * cosx ) /16384) + ((iBeta * sinx ) /16384));
		iIq = (((iBeta  * cosx )  /16384) - ((iAlpha * sinx ) /16384));

		iIdPeriod += iId;
		iIqPeriod += iIq;
		iCountDivFactor++;

		if(cTriggerPeriod & 0x01  || iCountDivFactor > 36000)  // timeout
		{
			cTriggerPeriod &= 0x01^0xFF;
			iIqPeriod2 = iIqPeriod/iCountDivFactor;
			iIdPeriod2 = iIdPeriod/iCountDivFactor;
			iIqPeriod  = 0;
			iIdPeriod  = 0;
			iCountDivFactor=0;
		}



		 //================ffqq field and torque control =================
		if(iStep1==0)
		{
			VsdRef1 = 0;
			VsqRef1 = 0;
			iFieldDiffSum  = 0;
			iTorqueDiffSum = 0;
		}

 		switch(iControlFOC)
		{
		case 0:
		case 1:   						 // Speed with angle_correction from field
				iFieldDiff1     = iFieldSet  - iId;
				iTorqueDiff1    = iTorqueF0  - iIq;
				break;

		case 2: iFieldDiff1     = iFieldSet  - iId;
				iTorqueDiff1    = iTorqueSet - iIq;  // <<<=== iTorqueSet
				break;
		case 3: break;
		default: iControlFOC = 0; break;
		}




		iFieldDiffSum -= iFieldDiff2;
		iFieldDiffSum += iFieldDiff1;
		iFieldDiff2    = iFieldDiffSum/2;

		//------------- calc hysteresis and set limit -----------------
		iTemp1 = iTorqueDiff1;
		if(iTemp1 < 0) iTemp1   = -iTemp1;
		if(iTemp1 < 20) iTemp1=0;		// hys

		if(iTemp1 > 150) iTemp1 = 150;	// limit
		if(iTorqueDiff1 < 0)iTorqueDiff1 = -iTemp1;
		else iTorqueDiff1 = iTemp1;

		iTorqueDiffSum -= iTorqueDiff2;
		iTorqueDiffSum += iTorqueDiff1;
		iTorqueDiff2    = iTorqueDiffSum/2;


		switch(iControlFOC)
		{
		case 0: if(iMotDirection < 0)
				{
				VsdRef1 =  512*32;
				VsqRef1 = -1200*32;
				}
		else
		{
			VsdRef1 = 512*32;
			VsqRef1 = +1200*32;
		}
				break;
		case 1:   								 // Speed with angle_correction from field
			//    VsqRef1 = iIqPeriod2 * 32;       //
				VsdRef1 += iFieldDiff2  /256;
				VsqRef1 += iTorqueDiff2 /256;
				break;


		case 2:	VsdRef1 += iFieldDiff2  /16;
				VsqRef1 += iTorqueDiff2 /4;								// FOC torque-control
				break;
		case 3: break;
		default: iControlFOC = 0; break;
		}


		VsdRef2 = VsdRef1/ 32;
		VsqRef2 = VsqRef1/ 32;




 //================= invers park transformation =====================
		VsaRef = VsdRef2 * cosx/16384   - VsqRef2 * sinx/16384;
		VsbRef = VsdRef2 * sinx/16384   + VsqRef2 * cosx/16384;
		iAngleInvPark  = arctg1(VsaRef,VsbRef);           			// from 0 - 4095



		switch(iLoopCount & 0x03)
		{
			case 0:
					iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
					iVectorInvPark = root_function(iVectorInvPark);
					break;

			case 1:
					iVectorCurrent = iAlpha * iAlpha + iBeta * iBeta;
					iVectorCurrent = root_function(iVectorCurrent);
					break;

			case 2: if(a1SquareMean)
						a1RMS = root_function(a1SquareMean);
						a1SquareMean = 0;
					break;

			case 3: if(a2SquareMean)
						a2RMS = root_function(a2SquareMean);
						a2SquareMean = 0;
					break;
		}



    //****** mm0 ********************************************************

		switch(iControlFOC)
		{
			case 0:
			case 1:
					if(iSpeedValueNew)
						SpeedControl();
					CalcUmotForSpeed();
					break;

			case 2:
					iUmotResult = iVectorInvPark/2;		// FOC torque-control
					break;
			case 3: break;
			default:
				iControlFOC = 0;
				break;
		}
		iPowerMotor = 6863 * iIqPeriod2;
		iPowerMotor /= 65536;
		iPowerMotor *= iActualSpeed;
		iPowerMotor /= 4944;




		//===========  if a1RMS > Limit block Umot and ramp ==============
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


		if(iActualSpeed >= iParRpmMotorMax)  // Limit Max Speed  +/- vvvvv
		{
			if(iUmotResult > iUmotLast) iUmotResult = iUmotLast;
		}
		//------------ iUmotMotor follows iUmotResult -----------
		iUmotLast = iUmotResult;
		if(iUmotResult > iUmotMotor) iUmotMotor++;
		if(iUmotResult < iUmotMotor) iUmotMotor--;




		//====== aa angle correction ======================================================
		iAngleFromRpm = iActualSpeed;
		if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;  // absolut value
		iAngleFromRpm *= iParAngleFromRPM;
		iAngleFromRpm /= 4000;

		//=========== calculate iAngleDiffPeriod  only for view ============================
	#ifdef DEBUG_commutation

		iAngleDiff = 0;
		if(iActualSpeed > 0)
		{
			iAngleDiff = iAngleCurrent - iAngleFromHall;
			if(iAngleDiff < 0)iAngleDiff += 4096;
		}
		if(iActualSpeed < 0)
		{
			iAngleDiff = iAngleFromHall - iAngleCurrent;
			if(iAngleDiff < 0)iAngleDiff += 4096;
		}
		iAngleDiffSum += iAngleDiff;
		iCountAngle++;
		if(cTriggerPeriod & 0x04 || iCountAngle > 36000)
		{
			cTriggerPeriod &= 0x04^0xFF;
			if(iCountAngle){
				iAngleDiffPeriod = iAngleDiffSum/iCountAngle;
				iAngleDiffSum    = 0;
				iCountAngle      = 0;
			}
		}//end if(cTriggerPeriod

	#endif
	//============================================================================================

		//============================= set angle for pwm ===========================================
		if (iMotDirection !=  0)
		{
			iAnglePWMFromHall = iAngleFromHall + iParAngleUser ;
			iAnglePWMFromHall &= 0x0FFF;
			iAnglePWMFromFOC  = iAngleInvPark + (4096 - iAngleDiffFOC) + iParAngleUser;
			iAnglePWMFromFOC  &= 0x0FFF;
			iAngleXXX = iAnglePWMFromFOC - iAnglePWMFromHall;
			iAnglePWM = iAnglePWMFromFOC;
		}

		if((iControlFOC) == 0)
		{
			if (iMotDirection <  0)
			{
				iAnglePWMFromHall = iAngleFromHall + iParAngleUser - iAngleFromRpm   + 2240;
				iAnglePWMFromHall &= 0x0FFF;
			}
			else if (iMotDirection >  0)
			{
				iAngleFromRpm    =0;
				iAnglePWMFromHall = iAngleFromHall + iParAngleUser    + iAngleFromRpm;
				iAnglePWMFromHall &= 0x0FFF;
				iAnglePWMFromFOC  = iAngleInvPark + (4096 - iAngleDiffFOC) + iParAngleUser;
				iAnglePWMFromFOC  &= 0x0FFF;
				iAngleXXX = iAnglePWMFromFOC - iAnglePWMFromHall;
			}
			iAnglePWM = iAnglePWMFromHall;
		}




	   //======================================================================================




		if(iStep1 == 0 )
		if(iMotHoldingTorque)
		{
			iAnglePWM = iAngleFromHall + iParAngleUser - 600;
		}

		iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff

		//======================================================================================================
		//======================================================================================================

		if(iStep1 == 0)
		{
			iUmotIntegrator = 0;
			iUmotP			= 0;
			iIqPeriod2		= 0;
			iIdPeriod2		= 0;
		}

	#ifdef DEBUG_commutation
/*
	 	 xscope_probe_data(0,iPhase1);
	 	 xscope_probe_data(1,a1RMS);
	 	 xscope_probe_data(2,iSetLoopSpeed);
	 	 xscope_probe_data(3,iAngleFromHall);
	 	 xscope_probe_data(4,iAngleCurrent);
	   	 xscope_probe_data(5,iUmotMotor);
	   	 xscope_probe_data(6,a1Square/1000);

		 xscope_probe_data(0,iPhase1);
		 xscope_probe_data(1,iActualSpeed);
		 xscope_probe_data(2,iSetLoopSpeed);
		 xscope_probe_data(3,iUmotIntegrator/256);
		 xscope_probe_data(4,iUmotMotor);
		 xscope_probe_data(5,iAngleDiffPeriod);
		 xscope_probe_data(6,iIqPeriod2);
*/

		 xscope_probe_data(0,iPhase1);
		 xscope_probe_data(1,iAngleCurrent);
		 xscope_probe_data(2,iAnglePWM);
		 xscope_probe_data(3,iAngleFromHall);
		 xscope_probe_data(4,iAngleInvPark);
		 xscope_probe_data(5,iAnglePWMFromHall);
		 xscope_probe_data(6,iAnglePWMFromFOC);
		 xscope_probe_data(7,iVectorCurrent);
		 xscope_probe_data(8,iVectorInvPark);
		 xscope_probe_data(9,iPhase2);

	#endif




		// pwm 13889 * 4 nsec = 55,556µsec  18Khz

		if(iControlFOC==3){
		iPwmIndexHigh += iPwmAddValue;
		iPwmIndexHigh &= 0x0FFFFFFF;
		iAnglePWM	   = iPwmIndexHigh/65536;
		}


		iIndexPWM = iAnglePWM >> 2;  // >> 4;
		sine_pwm( iIndexPWM, iUmotMotor, iMotHoldingTorque , pwm_ctrl, c_pwm_ctrl, iPwmOnOff );

		#ifdef DEBUG_commutation
			cLeds = 0;
			#define defpp1  0
			#define defpp2  64
			#define defpp3  128
			#define defpp4  192

			if(iIndexPWM > defpp1   && iIndexPWM < (defpp1+8)) cLeds = 0x01;
			if(iIndexPWM > defpp2   && iIndexPWM < (defpp2+8)) cLeds = 0x02;
			if(iIndexPWM > defpp3   && iIndexPWM < (defpp3+8)) cLeds = 0x04;
			if(iIndexPWM > defpp4   && iIndexPWM < (defpp4+8)) cLeds = 0x08;
			cLeds ^= 0xFF;
			p_ifm_shared_leds_wden <: cLeds;

	//		p_ifm_ext_d0 <: 0; // yellow
		#endif


	//======================== read current ============================
		#ifdef DC900
		{a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
		#endif
		 a1 = -a1;
		 a2 = -a2;
		// a3 = -a1 -a2;

		#ifdef DEBUG_commutation
			p_ifm_ext_d2 <: 1;
		#endif



		#ifdef DC100
		 {a1, a2, a3}  = get_adc_vals_calibrated_int16_ltc1408( adc );
			#ifdef DEBUG_commutation
		 	 cFlag |= 0x02; testport <: cFlag;  // oszi green C4
			#endif
		#endif

		#ifdef DEBUG_commutation
			#ifdef DC900
		 	 	 p_ifm_ext_d2 <: 0;
			#endif
		#endif

		SaveValueToArray();



		select
				{
					case c_motvalue :> cmd2:
						 if(cmd2 == 4){
							 c_motvalue :> iSetValueSpeed;
								if(iParDefSpeedMax > 0)
								{
									iSetValueSpeed *= iParRPMreference;
									iSetValueSpeed /= iParDefSpeedMax;
								}
								if(iSetValueSpeed > iParRpmMotorMax)
									iSetValueSpeed = iParRpmMotorMax;
									iSetInternSpeed = iSetValueSpeed * 65536;
							}


						 	else if(cmd2 >= 32 && cmd2 < 64)
							{  	c_motvalue <: iMotValue[cmd2-32];
								c_motvalue <: iMotValue[cmd2-31];
								c_motvalue <: iMotValue[cmd2-30];
								c_motvalue <: iMotValue[cmd2-29];
						    }


						else if(cmd2 >= 64 && cmd2 < 96)  { iTemp = (int) cmd2; c_motvalue <: iMotPar[iTemp-64]; 	}
						else if(cmd2 >= 96 && cmd2 < 128) { iTemp = (int) cmd2; c_motvalue :> iTemp1;  iMotPar[iTemp-96] = iTemp1; iUpdateFlag=1;}
						break;

					default:
						break;
				}// end select



		select
		{
			case c_commutation :> cmd1:
				if(cmd1 == 1){
					c_commutation :> iPwmOnOff;
				}
				else if(cmd1 == 2){
					c_commutation :> iMotHoldingTorque;
				}
				else if(cmd1 == 3){
					c_commutation :> iTorqueSet;
				}     // milliNewtonTorque
				else if(cmd1 == 4){
					c_commutation :> iSetValueSpeed;
					if(iParDefSpeedMax > 0)
					{
						iSetValueSpeed *= iParRPMreference;
						iSetValueSpeed /= iParDefSpeedMax;
					}
					if(iSetValueSpeed > iParRpmMotorMax)
						iSetValueSpeed = iParRpmMotorMax;
						iSetInternSpeed = iSetValueSpeed * 65536;
				}
				else if(cmd1 == 5)
				{
					c_commutation :> iControlFOC;
				}
				else if(cmd1 >= 32 && cmd1 < 64)  { iTemp = (int) cmd1; c_commutation <: iMotValue[iTemp-32]; 	}
				else if(cmd1 >= 64 && cmd1 < 96)  { iTemp = (int) cmd1; c_commutation <: iMotPar[iTemp-64]; 	}
				else if(cmd1 >= 96 && cmd1 < 128) { iTemp = (int) cmd1; c_commutation :> iTemp1;  iMotPar[iTemp-96] = iTemp1; iUpdateFlag=1;}
				break;

			default:
				break;
		}// end select

		if(iUpdateFlag)
		{ iUpdateFlag=0; SetParameterValue(); }

	}// end while(1)

}// end function






void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  do_adc_calibration_ad7949(c_adc);
	  comm_sine(c_adc, c_commutation, c_hall, c_pwm_ctrl, c_motvalue);
}


//======================== SPEED --- CONTROL ====================================
void SpeedControl()
{
	int iTemp1;
	int iHysteresis;

	idiffSpeed  = iSetLoopSpeed - iActualSpeed;
	idiffSpeed2 = idiffSpeed;

	iHysteresis  = iSetLoopSpeed;
	if(iHysteresis < 0) iHysteresis = -iHysteresis; 					  // absolut value
	iHysteresis *= iParHysteresisPercent;
	iHysteresis /= 100;

	if(idiffSpeed > 0)
	{
		if(idiffSpeed < iHysteresis)  idiffSpeed2=0;  					   // Hysteresis
		if(idiffSpeed2 > iParDiffSpeedMax) idiffSpeed2 = iParDiffSpeedMax; // Limit

		idiffSpeed2 *= idiffSpeed2;       // square
		idiffSpeed2 /= iParDiffSpeedMax;
	}

	if(idiffSpeed < 0)
	{
		if(idiffSpeed > -iHysteresis)  idiffSpeed2=0;
		if(idiffSpeed2 < -iParDiffSpeedMax) idiffSpeed2 = -iParDiffSpeedMax;

		iTemp1 = -idiffSpeed2;
		iTemp1 *= iTemp1;
		iTemp1 /= iParDiffSpeedMax;
		idiffSpeed2 = -iTemp1;
	}

	if(iSetLoopSpeed < 0) idiffSpeed2 = -idiffSpeed2;  			// invert if speed negativ
		if(iIntegralGain < (iParIntegralGain * 64)) iIntegralGain++;

		iUmotIntegrator += idiffSpeed2 * iIntegralGain / 256;
		iUmotP           = idiffSpeed2 * iParPropGain  /  32;

		//-------------- set limits -----------------------------------
		if(iUmotIntegrator > 0)
		if(iUmotIntegrator > (iParUmotIntegralLimit * 256))  iUmotIntegrator = (iParUmotIntegralLimit * 256);

		if(iUmotIntegrator < 0)
		if(iUmotIntegrator < -(iParUmotIntegralLimit * 256)) iUmotIntegrator = -(iParUmotIntegralLimit * 256);
}


void CalcUmotForSpeed()
{
	int iTemp;

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
	iUmotProfile = iUmotLinear;

	if(iUmotSquare > iUmotLinear) iUmotProfile = iUmotSquare;
	if(iUmotProfile > 4096)        iUmotProfile = 4096;
	//----------------------------------------------------------------
	iUmotResult  = iUmotProfile +  iUmotIntegrator/256  + iUmotP/256 ;
	iUmotResult += (iUmotBoost / 32);

	if(iUmotResult > 4096)   iUmotResult 	= 4096;
	if(iUmotResult < 0 )     iUmotResult 	= 0;
}

void CalcRampForSpeed(){
	int iTemp1,iTemp2;
	int iTemp3,iTemp4;

	iTemp1 = iSetSpeedRamp;  	if(iTemp1 < 0) iTemp1 = -iTemp1;
	iTemp2 = iSetInternSpeed;  if(iTemp2 < 0) iTemp2 = -iTemp2;
	iTemp3 = iActualSpeed;     if(iTemp3 < 0) iTemp3 = -iTemp3;

	if(iTemp1 < iTemp2)  // acceleration +/-
	{
	iTemp4 = iTemp1/65536 - iTemp3;
	  if(iTemp4 > 200) iRampBlock = 1;

	  if(iRampBlock==0)
	  iTemp1 += iRampAccValue;
	  if(iTemp1 > iTemp2) iTemp1 = iTemp2;

	  if(iRampAccValue < defRampMax) iRampAccValue+=8;
	}

	if(iTemp1 > iTemp2)  // deceleration +/-
	{
	  iTemp1 -= iRampDecValue;
	  if(iTemp1 < iTemp2) iTemp1 = iTemp2;

	  if(iRampDecValue < defRampMax) iRampDecValue += 8;
	}

	if(iMotDirection < 0) iSetSpeedRamp = -iTemp1; else iSetSpeedRamp = iTemp1;
	iSetLoopSpeed = iSetSpeedRamp/65536;
	iRampBlock = 0;

}//end CalcRampForSpeed

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
	iMotValue[0]  = iUmotProfile;
	iMotValue[1]  = iUmotIntegrator/256;
	iMotValue[2]  = iUmotP;
	iMotValue[3]  = iUmotMotor;
	iMotValue[4]  = iActualSpeed;
	iMotValue[5]  = iStep1;

	iMotValue[6]  = iControlFOC;
	iMotValue[7]  = iSetInternSpeed/65536;
	iMotValue[8]  = iSetLoopSpeed;
	iMotValue[9]  = idiffSpeed;
	iMotValue[10] = idiffSpeed2;
	iMotValue[11] = iMotHoldingTorque;

	iMotValue[12] = iAngleFromHall;
	iMotValue[13] = iAnglePWM;
	iMotValue[14] = iPowerMotor;
	iMotValue[15] = 0;
	iMotValue[16] = iAngleDiffPeriod;
	iMotValue[17] = iMotDirection;

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
	iMotValue[28] = iPinStateEncoder;
	iMotValue[29] = iPinStateHall;
	iMotValue[30] = iPositionEncoder;
	iMotValue[31] = iPositionAbsolut;
}





