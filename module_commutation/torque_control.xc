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
				if(iSetInternSpeed > 0   && iTorqueUser != 0){iMotDirection =   1; VsqRef1 = 4096;  iStep1 = 10;}
				if(iSetInternSpeed < 0   && iTorqueUser != 0){iMotDirection =  -1; VsqRef1 =-4096;  iStep1 = 10;}
				break;

				//==========================================
		case 10: iPwmOnOff 		 = 1;
				 if(iTorqueUser  ==0)     iStep1++;
				 if(iSetInternSpeed == 0) iStep1++;
				 iCountx =0;
				 break;

		case 11: iSetInternSpeed = 0;
			     if(iActualSpeed == 0)iStep1=0;
			     if(iCountx++ > 9000) iStep1=0;
		         break;
		//--------------- overcurrent --------------------------
		case 30:  iPwmOnOff		  = 0;			// error motor stop
				  iStep1++;
				  break;

		case 31:  iPwmOnOff		  =  0;
		 	 	  if(iControlFOC > 1){
		 	 	  if(iTorqueUser == 0 ) iStep1=0;
		 	 	  }
		 	 	  else
		 	 	  {
				  if(iSetLoopSpeed== 0)iStep1=0;     // motor is stopping
		 	 	  }
		 	 	  break;
		default:iStep1 = 0;
				break;
	}// end iStep1

	    CalcRampForSpeed();

		FOC_ClarkeAndPark();

		iTorqueSet = iTorqueUser;
		if(iTorqueSet < 0) iTorqueSet = -iTorqueSet;

		if(iSetInternSpeed < 0) iTorqueSet = -iTorqueSet;
		if(iSetInternSpeed == 0) iTorqueSet=0;


		//------------   diff = set - actual --------------
		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueSet - iIq;  // <<<=== iTorqueSet

		FOC_FilterDiffValue();
        //-------------------------------------------------

		//================== speed limit =========================================
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
        //========================================================================

		FOC_Integrator();


		FOC_InversPark();

		if(iSpeedValueIsNew) SpeedControl();

		iUmotResult = iVectorInvPark;		 // FOC torque-control

		//----------------------------------------------------------------
	//	iUmotResult  = iVectorInvPark +  iUmotIntegrator/256  + iUmotP/256 ;
	//	iUmotResult += (iUmotBoost / 256);

		if(iUmotResult > 4096) iUmotResult = 4096;

}
