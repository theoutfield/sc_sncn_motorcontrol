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

		iFieldIntegral += iFieldDiff2  /16;
		iFieldProp      = iFieldDiff2  /64;

		VsdRef1 = iFieldIntegral + iFieldProp;

		iTorqIntegral += iTorqueDiff2 / 16;
		iTorqProp      = iTorqueDiff2 / 64;

		VsqRef1 = iTorqIntegral + iTorqProp;


	//	VsdRef1 += iFieldDiff2  /32;
	//	VsqRef1 += iTorqueDiff2 /32;					// FOC torque-control

		FOC_InversPark();

		iUmotResult = iVectorInvPark/4;		 // FOC torque-control
		if(iUmotResult > 4096) iUmotResult = 4096;


}
