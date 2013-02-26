#include "varext.h"
#include "def.h"


void    function_SpeedControl()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iMotDirection    = 0;
				if(iSetInternSpeed > 0){iMotDirection =  1;  VsqRef1=  1024; VsdRef1 = 512; iTorqueF0 =  500;  iStep1= 1; }
				if(iSetInternSpeed < 0){iMotDirection = -1;  VsqRef1= -1024; VsdRef1 = 512; iTorqueF0 = -500;  iStep1= 1; }
				break;

		case 1:  iUmotBoost = iParUmotBoost * 32;
		         iUmotMotor = iParUmotStart;
				 iPwmOnOff	= 1;
				 iRampIntegrator = 0;
				 iStep1++;
				 break;

		case 2:	 iStep1++; break;
		case 3:  iStep1++; break;
		case 4:  iStep1++; break;

		case 5:  if(iUmotBoost > 0)iUmotBoost--;
				 if(iSetInternSpeed == 0) iStep1=11;  // motor stops
				 else
				 {
			     if(iSetInternSpeed < 0 && iSetSpeedRamp > 0)iStep1=20;	 // change direction from plus to minus
				 if(iSetInternSpeed > 0 && iSetSpeedRamp < 0)iStep1=20;  // change direction from minus to plus
				 }
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
		case 15: iStep1=0;     				// motor is stopping
					break;

		//----------------------------change direction  -----------------------
		case 20: iSetInternSpeed2 = iSetInternSpeed;
			     iSetInternSpeed = 0;
				 iCountx = 0;
                 iStep1++;
                 break;
		case 21: iSetInternSpeed = 0;
			     if(iSetSpeedRamp==0)  iStep1++;
		         break;
		case 22: iSetInternSpeed = 0;
			     if(iCountx++ > 500)  iStep1++;  // wait 27 msec
		         break;
		case 23: iSetInternSpeed = iSetInternSpeed2;
		         iStep1 = 0;
		         break;
		//------------------------------------------------------

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

		FOC_ClarkeAndPark();


		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueF0  - iIq;  // <<<=== iTorqueF0

		FOC_FilterDiffValue();

		VsdRef1 += iFieldDiff2  /256;
		VsqRef1 += iTorqueDiff2 /256;

		VsdRef2 = VsdRef1/ 32;
		VsqRef2 = VsqRef1/ 32;


		FOC_InversPark();


		if(iSpeedValueNew)SpeedControl();

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
	iTemp2 = iSetInternSpeed;   if(iTemp2 < 0) iTemp2 = -iTemp2;
	iTemp3 = iActualSpeed;      if(iTemp3 < 0) iTemp3 = -iTemp3;

	if(iStep1==0) iStepRamp = 0;
	switch(iStepRamp)
	{
	case 0: //iRampAccValue = defRampMin;
			//iRampDecValue = defRampMin;
			break;
	case 1: iSpeedSmoothed = iTemp1;
			iRampAccValue  = defRampMin;
			iStepRamp++;
			break;
	case 2: if(iRampAccValue < defRampMax) iRampAccValue+=8;
			else iStepRamp++;
			break;
	case 3: iSpeedSmoothed = iTemp1 - iSpeedSmoothed;
	        iStepRamp++;
	        break;
	case 4: if((iTemp2 - iTemp1) < iSpeedSmoothed) iStepRamp++;
			break;
	case 5: if(iRampAccValue > defRampMin) iRampAccValue-=8;
	        break;
	//------------ deceleration ----------------------------
	case 11: iSpeedSmoothed = iTemp1;
	         iRampDecValue = defRampMin;
	         iStepRamp++;
	         break;
	case 12: if(iRampDecValue < defRampMax) iRampDecValue += 8;
			 else iStepRamp++;
	         break;
	case 13: iSpeedSmoothed = iSpeedSmoothed - iTemp1;
	         iStepRamp++;
	         break;
	case 14: if((iTemp1 - iTemp2) < iSpeedSmoothed) iStepRamp++;
			 break;
	case 15: if(iRampDecValue > defRampMin) iRampDecValue-=8;
	         break;
	}



	if(iTemp1 < iTemp2)  // acceleration +/-
	{
	if(iStepRamp==0)    iStepRamp = 1;
	if(iStepRamp >= 11) iStepRamp = 1;

	iTemp4 = iTemp1/65536 - iTemp3;
	  if(iTemp4 > 200) iRampBlocked = 1;

	  if(iRampBlocked==0)
	  iTemp1 += iRampAccValue;
	  if(iTemp1 >= iTemp2) { iTemp1 = iTemp2; iStepRamp=0;}
	}

	if(iTemp1 > iTemp2)  // deceleration +/-
	{
	if(iStepRamp < 11) iStepRamp = 11;

	  iTemp1 -= iRampDecValue;
	  if(iTemp1 <= iTemp2) { iTemp1 = iTemp2; iStepRamp=0;}
	}

	if(iMotDirection < 0) iSetSpeedRamp = -iTemp1; else iSetSpeedRamp = iTemp1;
	iSetLoopSpeed = iSetSpeedRamp/65536;
	iRampBlocked = 0;

}//end CalcRampForSpeed
