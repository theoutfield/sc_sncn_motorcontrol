#include "varext.h"
#include "def.h"


void    function_SpeedControl()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;				// motor stop
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iMotDirection    = 0;
				iTorqueF0        = 0;
				VsqRef1          = 0;
				if(iSetInternSpeed != 0) iStep1++;
				break;

		case 1:  //======================== start motor ====================
			     iStep1++;
			     if(iSetInternSpeed > 0){iMotDirection =  1; VsdRef1=0; VsqRef1 =  4096;  iTorqueF0 =  500;   }
				 if(iSetInternSpeed < 0){iMotDirection = -1; VsdRef1=0; VsqRef1 = -4096;  iTorqueF0 = -500;   }
				 iPwmOnOff	= 1;
				 break;

		case 2:  iUmotBoost = iParUmotBoost * 256;
		         iUmotMotor = iParUmotStart;
				 iPwmOnOff	= 1;
				 iStep1++;
				 break;

		case 3:  iStep1++; break;
		case 4:  iStep1++; break;
		case 5:  if(iUmotBoost > 0)iUmotBoost--;
				 if(iSetInternSpeed == 0) iStep1=11;  // motor stops
				 else
				 {
			     if(iSetInternSpeed < 0 && iSetSpeedRamp > 0) iStep1=15;  // change direction from plus to minus
				 if(iSetInternSpeed > 0 && iSetSpeedRamp < 0) iStep1=18;  // change direction from minus to plus
				 }
				 break;

		case 11: iCountx = 0;						// motor is stopping
				 iTorqueF0 		  =	0;
				 iSetInternSpeed  = 0;
		         iStep1++;
		         break;

		case 12: iSetInternSpeed = 0;
			     if(iSetSpeedRamp == 0 && iCountx++ > 5000) iStep1=0;
		         break;
		//---------------------------------------------------------------------
		case 15: if(iSetSpeedRamp < 0) iStep1++;
		         break;
		case 16: iMotDirection = -1; VsdRef1=0; VsqRef1 = -4096;  iTorqueF0 = -500;
		         iStep1=2;
		         break;

		case 18: if(iSetSpeedRamp > 0) iStep1++;
		         break;
		case 19: iMotDirection = 1; VsdRef1=0; VsqRef1 = 4096;  iTorqueF0 = +500;
		         iStep1=2;
		         break;
		//--------------- overcurrent ----------------------------------
		case 30:  iPwmOnOff		  = 0;			     // error motor stop
				  if(iSetLoopSpeed== 0)iStep1=0;
				  break;

		default:  iStep1 = 0;
				  break;
	}

		//===========================================================
		CalcRampForSpeed();
		//============================== ramp calculation ===========

		FOC_ClarkeAndPark();


		iTorqueSet = iTorqueF0;

		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueF0  - iIq;  // <<<=== iTorqueF0

		FOC_FilterDiffValue();

		VsdRef1 += iFieldDiff2  /128;
		VsqRef1 += iTorqueDiff2 /128;



		FOC_InversPark();



		if(iSpeedValueIsNew) SpeedControl();

		CalcUmotForSpeed();
}// end function SpeedControl



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



	if(iTemp > iParRpmUmotMax) iTemp = iParRpmUmotMax; // set limit
	iUmotSquare  = iTemp * iTemp;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare *= 4096;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare += iParUmotStart;

	iUmotLinear = iTemp * 4096;
	iUmotLinear /= iParRpmUmotMax;

	iUmotProfile = iUmotLinear;

	if(iUmotSquare > iUmotLinear)  iUmotProfile = iUmotSquare;

	if(iUmotProfile > 4096)        iUmotProfile = 4096;
	//----------------------------------------------------------------
	iUmotResult  = iUmotProfile +  iUmotIntegrator/256  + iUmotP/256 ;
	iUmotResult += (iUmotBoost / 256);



	iTemp = iActualSpeed;	if(iTemp < 0) iTemp = -iTemp;
	if(iTemp > iParRpmMotorMax) iUmotRpmLimit++;
	else
	{if(iUmotRpmLimit > 0) iUmotRpmLimit--;}

	iUmotResult -=  iUmotRpmLimit;

	if(iUmotResult > 4096)   iUmotResult 	= 4096;
	if(iUmotResult < 0 )     iUmotResult 	= 0;
}




void CalcRampForSpeed(){


/*
 int iTemp;
	iSetLoopSpeed = iSetSpeedNew/65536;

	if(iSetLoopSpeed > 0){
		if(iSetLoopSpeed > iActualSpeed)
		{
			iTemp = iSetLoopSpeed - iActualSpeed;
			if(iTemp > 100)iRampBlocked=1;
		}
	}
*/



if(iRampBlocked==0)
{
  if(iSetInternSpeed >  iSetSpeedRamp) { iSetSpeedRamp += iMotPar[24];  if(iSetSpeedRamp > iSetInternSpeed)  iSetSpeedRamp = iSetInternSpeed;}

  if(iSetInternSpeed <  iSetSpeedRamp) { iSetSpeedRamp -= iMotPar[25];  if(iSetSpeedRamp < iSetInternSpeed)  iSetSpeedRamp = iSetInternSpeed;}
}


iRampBlocked = 0;


iSetSpeedSum -= iSetSpeedNew;
iSetSpeedSum += iSetSpeedRamp;

iSetSpeedNew = iSetSpeedSum;
if(iMotPar[26])
iSetSpeedNew /= iMotPar[26];   // smoothing factor

 iSetLoopSpeed = iSetSpeedNew/65536;

}//end CalcRampForSpeed


int iAngleStart=0;
int iCountGo;
int iEncoderStart;
int iEncoderDiff;
int iEncoderFirst;
int iEncoderSum;


void    function_SensorLessControl()
{
		switch(iStep1)
		{
		case 0: iPwmOnOff 		 = 0;				// PWM off
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iMotDirection    = 0;
				if(iSetInternSpeed != 0) iStep1++;
				iEncoderStart   	= iEncoderAngle;
				iEncoderFirst   	= iEncoderAngle;
				iAngleStart     &= 0x0FFF;
				iAngleSensorLessPWM = iAngleStart;
				iEncoderSum = 0;
				break;

		case 1: iStep1++; // start motor
				iCountGo=0;
				if(iSetInternSpeed > 0){iMotDirection =  1; VsdRef1=0; VsqRef1 =  4096;  iTorqueF0 =  500;   }
				if(iSetInternSpeed < 0){iMotDirection = -1; VsdRef1=0; VsqRef1 = -4096;  iTorqueF0 = -500;   }
				iPwmOnOff	= 1;
				break;

		case 2: if(iSetInternSpeed == 0) iStep1=11;     // motor stops from user

				if(iEncoderStart > 2048 && iEncoderAngle < 2048)
				iEncoderDiff = ((iEncoderAngle+4096) - iEncoderStart);
				else
				iEncoderDiff = (iEncoderAngle - iEncoderStart);
				iEncoderSum += iEncoderDiff;
				iEncoderStart = iEncoderAngle;

				if(iCountGo++ > 12000) iStep1++;       // 1 second
				if(iEncoderDiff  < 0){ iStep1=9; iCountGo=0; iAngleStart += 512;  return; }
				iAngleSensorLessPWM += iEncoderDiff;
				break;

		case 3: iCountGo=0;
				if(iEncoderSum > 50) { iEncoderSum=0; iStep1=2;}
				else iStep1=7;
				break;

		case 7: iPwmOnOff 		 = 0;
				iCountGo++;
				if(iCountGo > 9000) iStep1++; // half second
				break;

		case 8: iAngleStart    += 256;
			  	iStep1=0;
			    break;

		case 9: iPwmOnOff 		 = 0;
				iCountGo++;
				if(iCountGo > 9000) iStep1=8; // half second
				break;

		case 10: if(iSetInternSpeed == 0) iStep1=11;  // motor stops
			     break;

		case 11: iCountx = 0;						// motor is stopping
				 iTorqueF0 		  =	0;
				 iSetInternSpeed  = 0;
			     iStep1++;
			     break;

		case 12: iSetInternSpeed = 0;
				     if(iSetSpeedRamp == 0 && iCountx++ > 5000) iStep1=0;
			         break;

			case 15: if(iSetInternSpeed == 0) iStep1=0;  // motor stops
			         break;


			//--------------- overcurrent ----------------------------------
			case 30:  iPwmOnOff		  = 0;			     // error motor stop
					  if(iSetLoopSpeed== 0)iStep1=0;
					  break;

			default:  iStep1 = 0;
					  break;
		}

			//===========================================================
			CalcRampForSpeed();
			//============================== ramp calculation ===========

			// pwm 13889 * 4 nsec = 55,556µsec  18Khz
			// electrical: RefPeriod = 4096 * (1/18000)  = 227,56msec RefFreq= 4,394Hz => 263.67RPM
			// motor mechanical: 1000RPM  electrical 7000RPM => 7000/RefRPM = 26,548

			//iAngleRotorDiffCalculated  = iActualSpeed * 700;
			//	iAngleRotorDiffCalculated /= 26367;

/*
			iPwmAddValue  = iSetLoopSpeed * 700;
			iPwmAddValue *= 256;
			iPwmAddValue /= 26367;
			iPwmAddValue *= 256;

			iPwmIndexHigh += iPwmAddValue;
			iPwmIndexHigh &= 0x0FFFFFFF;  // normalize to 0x0FFF FFFF 0-4095

			iAngleSensorLessPWM	   = iPwmIndexHigh/65536;
*/

		//	iAngleSensorLessPWM   += iAngleStart;
			iAngleSensorLessPWM &= 0x0FFF;

		//	if(iSpeedValueNew) SpeedControl();

		//	CalcUmotForSpeed();


			if(iStep1 != 0)
			iUmotResult = iParUmotBoost + iParUmotStart;
}// end of funcion_SensorLessControl






