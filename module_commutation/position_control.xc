#include "varext.h"
#include "def.h"



int CheckIfNewPosition();
void CalcRampForPosition();


int CheckIfNewPosition()
{
int iTemp1;

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;
	if(iTemp1 < 0) iTemp1 = -iTemp1;
	if(iTemp1 < 5) return(1);

	iTemp1 = iPositionAbsolutNew - iPositionReferenz;

	if(iPositionAbsolutNew < iPositionAbsolut)
	return(10);
	else
	return(5);
}


int CalcSpeedForPosition()
{
int iTemp1;
int iSpeed;
int iDirection;

	if(iPositionAbsolutNew < iPositionAbsolut)	iDirection=-1;
	else iDirection=1;

    iTemp1 = iPositionAbsolutNew - 	iPositionAbsolut;
	if(iTemp1 == 0) return(0);

	if(iTemp1 < 0) iTemp1 = -iTemp1;

	iSpeed = iTemp1;
	if(iSpeed > iMotPar[29])  iSpeed = iMotPar[28];       // defParPositionSpeedMax
	if(iSpeed < iMotPar[29])  iSpeed = iMotPar[29];       // defParPositionSpeedMin

	if(iDirection < 0)	iSpeed = -iSpeed;

return (iSpeed*65536);
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
				iPositionReferenz = iPositionAbsolut;
				iStep1++;
				break;

		case 1: iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iSetInternSpeed  = 0;
				iMotDirection    = 0;
			    iStep1 = CheckIfNewPosition();
		        break;

		case 2: if(--iCountx <= 0)
				{
				iPositionReferenz = iPositionAbsolutNew;
				iStep1=1;
				}
				break;
//-------------------------------------------------------



		case 5:	iSetInternSpeed = CalcSpeedForPosition();
			    iUmotBoost      = iParUmotBoost * 32;
				iMotDirection   = 1;     VsqRef1=  1024;  iTorqueF0 =  500;
		        iUmotMotor      = iParUmotStart;
				iPwmOnOff	    = 1;
				iPulsCountAcc   = iPositionAbsolut;
				iStep1++;
				break;

		case 6:	if(iUmotBoost > 0)iUmotBoost--;
		        iSetInternSpeed = CalcSpeedForPosition();
		        if(iPositionAbsolut > (iPositionAbsolutNew-10)) iStep1++;
		        break;

		case 7:	iSetInternSpeed = 0;
			    iCountx 		= 1000;
				iStep1=2;
			    break;


	 //--------------------------------------------------

//----------------------------------------------------------
		case 10: iSetInternSpeed = CalcSpeedForPosition();
				 iUmotBoost = iParUmotBoost * 32;
				 iMotDirection = -1;  VsqRef1= -1024; VsdRef1 = 512; iTorqueF0 = -500;
		         iUmotMotor = iParUmotStart;
				 iPwmOnOff	= 1;
				 iStep1++;
				 break;

		case 11: if(iUmotBoost > 0)iUmotBoost--;
        		 iSetInternSpeed = CalcSpeedForPosition();
		         if(iPositionAbsolut  < (iPositionAbsolutNew+10)) iStep1++;
		         break;

		case 12: iSetInternSpeed = 0;
		         iStep1=2;
			     break;

		//------------------------------------------------------





		//--------------- overcurrent --------------------------
		case 30:  iPwmOnOff		  = 0;					// error motor stop
				  iStep1++;
				  break;

		case 31:  iPwmOnOff		  =  0;
		 	 	  if(iControlFOC > 1){
		 	 	  if(iTorqueSet == 0 ) iStep1=0;
		 	 	  }
		 	 	  else
		 	 	  {
				  if(iSetLoopSpeed== 0)iStep1=0;     	// motor is stopping
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

if(iRampBlocked==0)
{
  if(iSetInternSpeed >  iSetSpeedRamp) { iSetSpeedRamp += iMotPar[24]/8;  if(iSetSpeedRamp > iSetInternSpeed)  iSetSpeedRamp = iSetInternSpeed;}

  if(iSetInternSpeed <  iSetSpeedRamp) { iSetSpeedRamp -= iMotPar[25]/8;  if(iSetSpeedRamp < iSetInternSpeed)  iSetSpeedRamp = iSetInternSpeed;}
}
iRampBlocked = 0;


iSetSpeedSum -= iSetSpeedNew;
iSetSpeedSum += iSetSpeedRamp;

iSetSpeedNew = iSetSpeedSum;
if(iMotPar[26])
iSetSpeedNew /= iMotPar[26];   // smoothing factor

 iSetLoopSpeed = iSetSpeedNew/65536;

}//end CalcRampForSpeed
