#include "varext.h"
#include "def.h"

#define defSpeedMax  1000*65536
#define defSpeedMin   150*65536

int CheckIfNewPosition();
void CalcRampForPosition();


int CheckIfNewPosition()
{
int iTemp1;

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;
	if(iTemp1 < 0) iTemp1 = -iTemp1;
	if(iTemp1 < 5) return(0);

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;

	if(iTemp1 > 0 )
	{
	iSetInternSpeed2 = iTemp1 *65536;
	if(iSetInternSpeed2 > defSpeedMax) iSetInternSpeed2= defSpeedMax;
	if(iSetInternSpeed2 < defSpeedMin) iSetInternSpeed2= defSpeedMin;

	iPositionAcc     = iPositionReferenz   + iTemp1/3;

	iTemp1 /= 3;
	if(iTemp1 > 150) iTemp1 = 150;
	iPositionDec     = iPositionAbsolutNew - iTemp1;


	return(1);
	}

	if(iTemp1 < 0 )
	{
	iSetInternSpeed2 = iTemp1 *65536;
	if(iSetInternSpeed2 < -defSpeedMax) iSetInternSpeed2=-defSpeedMax;
	if(iSetInternSpeed2 > -defSpeedMin) iSetInternSpeed2=-defSpeedMin;
	//iTemp1 = -iTemp1;
	iPositionDec     = iPositionAbsolutNew - iTemp1;

	return(10);
	}

return(0);
}


void function_PositionControl()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iSetInternSpeed  = 0;
				iMotDirection    = 0;
				iStep1 = CheckIfNewPosition();
				break;

		case 1:	iSetInternSpeed = iSetInternSpeed2;
			    iUmotBoost      = iParUmotBoost * 32;
				iMotDirection   = 1;  VsqRef1=  1024; VsdRef1 = 512; iTorqueF0 =  500;
		        iUmotMotor      = iParUmotStart;
				iPwmOnOff	    = 1;
				iRampIntegrator = 0;
				iPulsCountAcc   = iPositionAbsolut;
				iStep1++;
				break;

		case 2: iStep1++;
				iSetInternSpeed = iSetInternSpeed2;
			    break;

		case 3:	if(iUmotBoost > 0)iUmotBoost--;
		        iSetInternSpeed = iSetInternSpeed2;

		        if(iActualSpeed >= iSetLoopSpeed)
		        {
		       	iPulsCountAcc = iPositionAbsolut;
		       	iStep1++;
		        }
		        if(iPositionAbsolut > (iPositionAbsolutNew-10)) iStep1++;
		        break;

		case 4: iSetInternSpeed = iSetInternSpeed2;
			    if(iPositionAbsolut > (iPositionAbsolutNew-10)) iStep1++;
				if(iPositionAbsolut > iPositionDec) iStep1++;
				break;

		case 5: iSetInternSpeed =	defSpeedMin;
			    if(iPositionAbsolut > (iPositionAbsolutNew-10)) iStep1++;
		        break;

		case 6:	iSetInternSpeed = 0;
			    iCountx 		= 1000;
				iStep1++;
			    break;

		case 7:  if(--iCountx <= 0)
				 {
				 iPositionReferenz = iPositionAbsolutNew;
				 iStep1=0;
				 }
				 break;

//----------------------------------------------------------
		case 10: if(iSetInternSpeed2 < -defSpeedMax)  iSetInternSpeed2= -defSpeedMax;
				 if(iSetInternSpeed2 > defSpeedMin)   iSetInternSpeed2= -defSpeedMin;
				 iSetInternSpeed = iSetInternSpeed2;

				 iUmotBoost = iParUmotBoost * 32;
				 iMotDirection = -1;  VsqRef1= -1024; VsdRef1 = 512; iTorqueF0 = -500;
		         iUmotMotor = iParUmotStart;
				 iPwmOnOff	= 1;
				 iRampIntegrator = 0;
				 iStep1++;
				 break;

		case 11: if(iUmotBoost > 0)iUmotBoost--;
		         iSetInternSpeed = iSetInternSpeed2;
		         if(iPositionAbsolutNew >= iPositionAbsolut) iStep1++;
		         break;

		case 12: iSetInternSpeed = 0;
		         iStep1=3;
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

		default: iStep1 = 0;
				 break;
	}// end iStep1


		//===========================================================
		CalcRampForPosition();
		//============================== ramp calculation ===========

		FOC_ClarkeAndPark();


		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueF0  - iIq;  // <<<=== iTorqueF0

		VsdRef1 += iFieldDiff2  /256;
		VsqRef1 += iTorqueDiff2 /256;

		FOC_FilterDiffValue();

		VsdRef1 += iFieldDiff2  /256;
		VsqRef1 += iTorqueDiff2 /256;

		FOC_InversPark();

		if(iSpeedValueNew)SpeedControl();

		CalcUmotForSpeed();

}

void CalcRampForPosition(){

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
	case 2: if(iRampAccValue < (defRampMax/4)) iRampAccValue+=8;
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
	case 12: if(iRampDecValue < (defRampMax/4)) iRampDecValue += 8;
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

