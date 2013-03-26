#include "varext.h"
#include "def.h"
#include "dc_motor_config.h"

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
				iFieldIntegrator   = 0;
				if(iSetUserSpeed > 0   && iTorqueUser != 0){iMotDirection =   1; iTorqueUmotIntegrator = 4096;  iStep1 = 10;}
				if(iSetUserSpeed < 0   && iTorqueUser != 0){iMotDirection =  -1; iTorqueUmotIntegrator =-4096;  iStep1 = 10;}
				break;

				//==========================================
		case 10: iPwmOnOff 		 = 1;
				// if(iTorqueUser  ==0)     iStep1++;
				 if(iSetUserSpeed == 0) iStep1++;
				 iCountx =0;
				 break;

		case 11: iSetUserSpeed   = 0;
			     if(iActualSpeed == 0)iStep1=0;
			     if(iCountx++ > 18000) iStep1=0;
		         break;
		//--------------- overcurrent --------------------------
		case 30:  iPwmOnOff		  = 0;			// error motor stop
				  iStep1++;
				  break;

		case 31:  iPwmOnOff		  =  0;
				  if(iSetSpeed== 0)iStep1=0;     // motor is stopping
		 	 	  break;
		default:iStep1 = 0;
				break;
	}// end iStep1


		FOC_ClarkeAndPark();

		iTorqueSet = iTorqueUser;
		if(iTorqueSet < 0) iTorqueSet = -iTorqueSet;

		if(iSetUserSpeed < 0)  iTorqueSet = -iTorqueSet;
		if(iSetUserSpeed == 0) iTorqueSet=0;


		//------------   diff = set - actual --------------
		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueSet - iIq;  // <<<=== iTorqueSet

		FOC_FilterDiffValue();
        //-------------------------------------------------


		//================== speed limit =========================================
		/*
		if(iActualSpeed > 0)
		{
		if(iActualSpeed > (defParRpmMotorMax+100))      iTorqueLimit = iTorqueIntegral;

		if(iActualSpeed < (defParRpmMotorMax-200))      iTorqueLimit += 64;
		if(iTorqueLimit > TORQUE_INTEGRATOR_MAX) iTorqueLimit = TORQUE_INTEGRATOR_MAX;
		}

		if(iActualSpeed < 0)
		{
		if(iActualSpeed < (-defParRpmMotorMax-100))     iTorqueLimit = -iTorqueIntegral;

		if(iActualSpeed > (-defParRpmMotorMax+200))     iTorqueLimit += 64;
		if(iTorqueLimit > TORQUE_INTEGRATOR_MAX) iTorqueLimit = TORQUE_INTEGRATOR_MAX;
		}
		*/
        //========================================================================


	    CalcRampForSpeed();

	    iUmotProfile  = CalcUmotProfile();

	    if(iSpeedValueIsNew) CalcDiffSpeed();

		FOC_Integrator();

		FOC_InversPark();

		iUmotResult = iVectorInvPark/2;		 // FOC torque-control

		if(iUmotResult > 4096) iUmotResult = 4096;


		//============================= set angle for pwm ============================================
			if (iMotDirection !=  0)
			{
				iAnglePWMFromFOC  = iAngleInvPark + (3076 + iParAngleUser);
				iAnglePWMFromFOC  &= 0x0FFF;
				iAnglePWM         = iAnglePWMFromFOC;
			}


}
