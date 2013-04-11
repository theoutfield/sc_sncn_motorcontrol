#include "varext.h"
#include "def.h"
#include "dc_motor_config.h"
#ifndef new

void    function_TorqueControl()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 		= 0;
				iIntegralGain    		= 0;
				iUmotProfile       		= 0;
				iUmotResult        		= 0;
				iSpeedIntegrator    	= 0;
				iTorqueUmotIntegrator 	= 0;
				iFieldIntegrator      	= 0;
				iMotDirection      		= 0;
				iTorqueLimit 	 		= TORQUE_INTEGRATOR_MAX * 4;
				if(iSpeedSetUser > 0   && iTorqueUser != 0){iMotDirection =   1; iTorqueUmotIntegrator = 4096;  iStep1 = 5;}
				if(iSpeedSetUser < 0   && iTorqueUser != 0){iMotDirection =  -1; iTorqueUmotIntegrator =-4096;  iStep1 = 5;}
				break;
				//=========================================================================
		case 5:  iPwmOnOff 		 = 1;
			  	 if(iSpeedSetUser == 0) iStep1=20;
			  	 iCountx =0;
			  	 break;

		case 20: if(iActualSpeed == 0) iStep1=0;
			     if(iCountx++ > 18000) iStep1=0;
		         break;

		//--------------- overcurrent --------------------------
		case 30:  iPwmOnOff		  = 0;			// error motor stop
				  iStep1++;
				  break;

		case 31:  iPwmOnOff		  =  0;
				  if(iSpeedSetUser == 0)iStep1=0;     // motor is stopping
		 	 	  break;
		default:iStep1 = 0;		break;
	}// end iStep1


		FOC_ClarkeAndPark();

		//------------- torque from user always positive -----------
		iTorqueSet = iTorqueUser;
		if(iTorqueSet < 0) iTorqueSet = -iTorqueSet;
		if(iSpeedSetUser < 0)   iTorqueSet = -iTorqueSet;
		if(iSpeedSetUser == 0)  iTorqueSet = 0;
		//------------------------------------------------


		//------------   diff = set - actual --------------
		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueSet - iIq;  // <<<=== iTorqueSet

		FOC_FilterDiffValue();
        //------------------------------------------------------------------------

	    CalcRampForSpeed();

	    iUmotProfile  = CalcUmotProfile();

	    if(iSpeedValueIsNew)
	    {
	    CalcDiffSpeed();
	    }

	    FOC_Integrator();

		FOC_InversPark();

		iUmotResult = iVectorInvPark/4   + iUmotProfile;		 // FOC torque-control

		if(iUmotResult > 4096) iUmotResult = 4096;

		//============================= set angle for pwm ============================================
			if (iMotDirection !=  0)
			{
				iAnglePWMFromFOC  = iAngleInvPark + 3720 + iParAngleUser;
				iAnglePWMFromFOC  &= 0x0FFF;
				iAnglePWM         = iAnglePWMFromFOC;
			}
}

#endif

