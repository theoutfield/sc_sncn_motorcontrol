#include "varext.h"
#include "def.h"


void    function_TorqueControl()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSetSpeedRamp    = 0;
				iMotDirection    = 0;
				if(iTorqueSet > 0){iMotDirection =   1; VsqRef1 = 4096;  iStep1 = 10;}
				if(iTorqueSet < 0){iMotDirection =  -1; VsqRef1 =-4096;  iStep1 = 10;}
				break;

				//==========================================
		case 10: iPwmOnOff 		 = 1;
				 if(iTorqueSet  ==0)iStep1++;
				 break;

		case 11: if(iActualSpeed == 0)iStep1=0;
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
		default:iStep1 = 0;
				break;
	}// end iStep1


		FOC_ClarkeAndPark();


		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueSet - iIq;  // <<<=== iTorqueSet

		FOC_FilterDiffValue();

		VsdRef1 += iFieldDiff2  /16;
		VsqRef1 += iTorqueDiff2 /4;					// FOC torque-control

#define defVsqRef1Max 220000

		if(VsqRef1 > 0)
		{
        if(VsqRef1 > defVsqRef1Max)VsqRef1 = defVsqRef1Max;         // Limit
		}

		if(VsqRef1 < 0)
		{
        if(VsqRef1 < -defVsqRef1Max)VsqRef1 = -defVsqRef1Max;         // Limit
		}


		FOC_InversPark();


		iUmotResult = iVectorInvPark/2;		// FOC torque-control
		if(iUmotResult > 4096) iUmotResult = 4096;


}
