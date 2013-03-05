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

		case 1:  // start motor
			     iStep1++;
			     if(iSetInternSpeed > 0){iMotDirection =  1; VsdRef1=0; VsqRef1 =  4096;  iTorqueF0 =  500;   }
				 if(iSetInternSpeed < 0){iMotDirection = -1; VsdRef1=0; VsqRef1 = -4096;  iTorqueF0 = -500;   }
				 iPwmOnOff	= 1;
				 break;

		case 2:  iUmotBoost = iParUmotBoost * 32;
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
			     if(iSetInternSpeed < 0 && iSetSpeedRamp > 0) iStep1=20;  // change direction from plus to minus
				 if(iSetInternSpeed > 0 && iSetSpeedRamp < 0) iStep1=25;  // change direction from minus to plus
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
		//----------------------------change direction  -----------------------
		case 20: iTorqueF0 		  =	0;
			     iSetInternSpeed  = 0;
				 iCountx = 0;
                 iStep1++;
                 break;
		case 21: iSetInternSpeed = 0;
			     if(iSetSpeedRamp==0)  iStep1++;
			 	 if(VsqRef1 < 4096) VsqRef1 = 4096;
		         break;
		case 22: iSetInternSpeed = 0;
		 	 	 iUmotIntegrator = 0;
			     if(iCountx++ > 5000)  iStep1 = 23;  // wait 27 msec
			 	 if(VsqRef1 < 4096) VsqRef1 = 4096;
			 	 if(iActualSpeed < 50) iStep1 = 23;
		         break;


		case 23: iIntegralGain    = 0;
				 iUmotProfile     = 0;
			     CalcSetInternSpeed(iMotCommand[0]);
				 iStep1 = 1;		// restart motor
				 break;
		//---------------------------------------------------------
		case 25: iTorqueF0 		  =	0;
			     iSetInternSpeed  = 0;
				 iCountx = 0;
                 iStep1++;
                 break;
		case 26: iSetInternSpeed = 0;
			     if(iSetSpeedRamp==0)  iStep1++;
			 	 if(VsqRef1 > -4096) VsqRef1 = -4096;
		         break;
		case 27: iSetInternSpeed = 0;
		 	 	 iUmotIntegrator = 0;
			     if(iCountx++ > 5000)  iStep1=28;  // wait 27 msec
			     if(VsqRef1 > -4096) VsqRef1 = -4096;
			     if(iActualSpeed > -50) iStep1 = 28;
		         break;
		case 28: iIntegralGain    = 0;
				 iUmotProfile     = 0;
			     CalcSetInternSpeed(iMotCommand[0]);
				 iStep1 = 1;		// restart motor
				 break;

		//--------------- overcurrent -----------------------------
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

		if(iSpeedValueNew) SpeedControl();

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

	if(iTemp > iParSpeedKneeUmot) iTemp = iParSpeedKneeUmot; // set limit
	iUmotSquare  = iTemp * iTemp;
	iUmotSquare /= iParSpeedKneeUmot;
	iUmotSquare *= 4096;
	iUmotSquare /= iParSpeedKneeUmot;
	iUmotSquare += iParUmotSocket;

	iUmotLinear = iTemp * 4096;
	iUmotLinear /= iParSpeedKneeUmot;
	iUmotProfile = iUmotLinear;

	if(iUmotSquare > iUmotLinear)  iUmotProfile = iUmotSquare;
	if(iUmotProfile > 4096)        iUmotProfile = 4096;
	//----------------------------------------------------------------
	iUmotResult  = iUmotProfile +  iUmotIntegrator/256  + iUmotP/256 ;
	iUmotResult += (iUmotBoost / 32);

	if(iUmotResult > 4096)   iUmotResult 	= 4096;
	if(iUmotResult < 0 )     iUmotResult 	= 0;
}




void CalcRampForSpeed(){

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
