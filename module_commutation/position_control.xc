#include "varext.h"
#include "def.h"

void function_PositionControl()  // ppc
{
int iTemp;

#define defSpeedMax 1000*65536
#define defSpeedMin   50*65536

	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iSetInternSpeed  = 0;
				iMotDirection    = 0;
				iTemp = iHallPositionAbsolutNew - iHallPositionReferenz;
				if(iTemp > 0 ) { iSetInternSpeed2 = iTemp *65536; iStep1++;  }
				if(iTemp < 0 ) { iSetInternSpeed2 = iTemp *65536; iStep1=10; }
				break;

		case 1:  if(iSetInternSpeed2 > defSpeedMax) iSetInternSpeed2=defSpeedMax;
				 if(iSetInternSpeed2 < defSpeedMin) iSetInternSpeed2=defSpeedMin;
				 iSetInternSpeed = iSetInternSpeed2;
			     iUmotBoost      = iParUmotBoost * 32;
				 iMotDirection   = 1;  VsqRef1=  1024; VsdRef1 = 512; iTorqueF0 =  500;
		         iUmotMotor      = iParUmotStart;
				 iPwmOnOff	     = 1;
				 iRampIntegrator = 0;
				 iHallPulsCountAcc = iHallPositionAbsolut;
				 iStep1++;
				 break;

		case 2:	 if(iUmotBoost > 0)iUmotBoost--;
		         iSetInternSpeed = iSetInternSpeed2;
		         if(iRampAccValue >= defRampMax)
		         {
		        	 iHallPulsCountAcc = iHallPositionAbsolut -  iHallPulsCountAcc;
		        	 iStep1++;
		         }
		         if(iHallPositionAbsolutNew <= (iHallPositionAbsolut-10)) iStep1++;
		         break;

		case 3:  iSetInternSpeed = iSetInternSpeed2;
			     if(iHallPositionAbsolutNew <= (iHallPositionAbsolut-10)) iStep1++;
				 if(iHallPositionAbsolutNew <= (iHallPositionAbsolut-iHallPulsCountAcc)) iStep1++;
				 break;


		case 4:  iSetInternSpeed=0;
				 iCountx = 1000;
				 iStep1++;
			     break;

		case 5:  if(--iCountx <= 0)
				 {
				 iHallPositionReferenz = iHallPositionAbsolutNew;
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
		         if(iHallPositionAbsolutNew >= iHallPositionAbsolut) iStep1++;
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

		default:
				#ifdef DEBUG_commutation
					printstr("error\n");
				#endif
				iStep1 = 0;
				break;
	}// end iStep1


		//===========================================================
		CalcRampForSpeed();
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
