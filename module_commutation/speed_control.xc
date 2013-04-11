#include "varext.h"
#include "def.h"
#include "dc_motor_config.h"
#ifndef new

void FOC_Speed_Integrator();
void RampAccPlus();





void    function_UmotControl() {        // F0
       	switch(iStep1)
		 {
		  case 0: iPwmOnOff 	 = 0;				// motor stop
		 		iUmotResult      = 0;
		 		if(iSpeedSetUser != 0) iStep1++;
		        break;

		  case 1: iPwmOnOff 		 = 1;
			      if(iUmotResult == 0) iStep1=0;
                  break;
		 }// end switch

       	//---- iSpeedSetUser = iMotCommand[0] * 65536

        if(iSpeedSetUser > 0)  	 iAnglePWM= iAngleRotor + iParAngleUser;
        if(iSpeedSetUser < 0)  	 iAnglePWM= iAngleRotor + iParAngleUser + 2048;
        if(iSpeedSetUser == 0)   iAnglePWM= iAngleLast;

        iUmotResult = iMotCommand[0];		// umot control
        if(iUmotResult < 0) iUmotResult = -iUmotResult;
}//=========================== end  function_UmotControl ================




void    function_SpeedControl_F1()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;				// motor stop
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSpeedIntegrator = 0;
				iMotDirection    = 0;
				iUmotBoost       = 0;
				if(iSpeedSetUser != 0) iStep1++;
				break;

		case 1:  //======================== start motor ====================
			     iStep1++;
				 iPwmOnOff	= 1;
			     if(iSpeedSetUser > 0){iMotDirection =  1;   }
				 if(iSpeedSetUser < 0){iMotDirection = -1;   }
		         iUmotBoost = iParUmotBoost * 256;
		         iUmotMotor = iParUmotStart;
				 break;

		case 2:  iCountx = 0;
			     if(iUmotBoost > 0)iUmotBoost--;
				 if(iSpeedSetUser == 0) iStep1=5;          // motor stops
				 break;

		case 5:  iCountx++;
			     if(iSpeedIntegrator == 0) iStep1=0;
			     if(iCountx > 36000)       iStep1=0;
		         break;
		//--------------- overcurrent ----------------------------------
		case 30:  iPwmOnOff		  = 0;			     // error motor stop
				  if(iSpeedIntegrator== 0)iStep1=0;
				  break;
		default:  iStep1 = 0;  break;
	}

		//===========================================================
		CalcRampForSpeed();
		//============================== ramp calculation ===========

		if(iSpeedValueIsNew) SpeedControl();

		CalcUmotForSpeed();

		iUmotResult  = iUmotProfile +  iUmotIntegrator/256  + iUmotP/256 ;
		iUmotResult += (iUmotBoost / 256);

		if(iUmotResult > 4096)   iUmotResult 	= 4096;
		if(iUmotResult < 0 )     iUmotResult 	= 0;

		//============================= set angle for pwm ============================================
		if (iMotDirection >  0)
		{
		iAnglePWM= iAngleRotor + iParAngleUser +400;
		}

		if (iMotDirection <  0)
		{
		iAnglePWM  = iAngleRotor + (2048 + iParAngleUser + 942);
		}

}//==== end function SpeedControl =======






void    function_SpeedControl_F2()
{
	switch(iStep1)
	{
		case 0: iPwmOnOff 		 = 0;				// motor stop
				iIntegralGain    = 0;
				iUmotProfile     = 0;
				iUmotResult      = 0;
				iSpeedIntegrator    = 0;
				iMotDirection    = 0;
				iTorqueF0        = 0;
				if(iSpeedSetUser != 0) iStep1++;
				break;

		case 1:  //======================== start motor ====================
			     iStep1++;
			     if(iSpeedSetUser > 0){iMotDirection =  1; iFieldIntegrator=0; iTorqueUmotIntegrator =  4096;  iTorqueF0 =  500;   }
				 if(iSpeedSetUser < 0){iMotDirection = -1; iFieldIntegrator=0; iTorqueUmotIntegrator = -4096;  iTorqueF0 = -500;   }
				 iPwmOnOff	= 1;
				 break;

		case 2:  iUmotBoost = iParUmotBoost * 256;
		         iUmotMotor = iParUmotStart;
				 iPwmOnOff	= 1;
				 iStep1++;
				 break;

		case 3:  iStep1++; break;
		case 4:  iStep1++; break;
		case 5:  if(iUmotBoost > 0)iUmotBoost--;
				 iCountx = 0;
				 if(iSpeedSetUser == 0) iStep1=11;  // motor stops
				 else
				 {
			     if(iSpeedSetUser < 0 && iSpeedIntegrator > 0) iStep1=15;  // change direction from plus to minus
				 if(iSpeedSetUser > 0 && iSpeedIntegrator < 0) iStep1=18;  // change direction from minus to plus
				 }
				 break;

		case 11:  iCountx++;
	     	 	 if(iSpeedIntegrator == 0) iStep1=0;
	     	 	 if(iCountx > 9000)        iStep1=0;
				 iTorqueF0 		  =	0;  // motor is stopping
		         break;


		//---------------------------------------------------------------------
		case 15: if(iSpeedIntegrator < 0) iStep1++;
		         break;
		case 16: iMotDirection = -1; iFieldIntegrator=0; iTorqueUmotIntegrator = -4096;  iTorqueF0 = -500;
		         iStep1=2;
		         break;

		case 18: if(iSpeedIntegrator > 0) iStep1++;
		         break;
		case 19: iMotDirection = 1; iFieldIntegrator=0; iTorqueUmotIntegrator = 4096;  iTorqueF0 = +500;
		         iStep1=2;
		         break;
		//--------------- overcurrent ----------------------------------
		case 30:  iPwmOnOff		  = 0;			     // error motor stop
				  if(iSpeedIntegrator== 0)iStep1=0;
				  break;

		default:  iStep1 = 0;
				  break;
	}

		//===========================================================
		CalcRampForSpeed();
		//============================== ramp calculation ===========

		FOC_ClarkeAndPark();

		if(iSpeedSetUser < 0) iTorqueF0 = -500;
		if(iSpeedSetUser > 0) iTorqueF0 =  500;

		iTorqueSet = iTorqueF0;



		//------------   diff = set - actual --------------
		iFieldDiff1     = iFieldSet  - iId;
		iTorqueDiff1    = iTorqueF0  - iIq;  // <<<=== iTorqueF0

		FOC_FilterDiffValue();
       //-------------------------------------------------

		FOC_Speed_Integrator();


		FOC_InversPark();

		if(iSpeedValueIsNew) SpeedControl();

		CalcUmotForSpeed();

		//============================= set angle for pwm ============================================
		if (iMotDirection !=  0)
		{
		iAnglePWMFromFOC  = iAngleInvPark + (3076 + iParAngleUser);
		iAnglePWMFromFOC  &= 0x0FFF;
		iAnglePWM         = iAnglePWMFromFOC;
		}

}// end function SpeedControl





void FOC_Speed_Integrator(){
int iTemp1;
#define defREFERENZFACTOR  64

	//---------------- field --------------------------


	iFieldIntegrator += iFieldDiff2  /8;

	iFieldProp      = iFieldDiff2;

	if(iFieldIntegrator >  16000)  iFieldIntegrator =  16000;
	if(iFieldIntegrator < -16000)  iFieldIntegrator = -16000;

	iFieldReferenz = (iFieldIntegrator + iFieldProp) /defREFERENZFACTOR;
	//-------------------------------------------------------------------


	//-------------- torque ---------------------------------------------
	iTemp1 = iTorqueDiff2;

	if(iSpeedIntegrator > 0)
	{
    if(iActualSpeed > iSpeedIntegrator/65536)
    	if(iTemp1 > 0) iTemp1=0;
 //   if(iSpeedValueIsNew) iTorqueUmotIntegrator += idiffSpeed2;
	}


    if(iSpeedIntegrator < 0)
    {
       if(iActualSpeed < (iSpeedIntegrator/65536))
    	    	if(iTemp1 < 0) iTemp1=0;
  //     if(iSpeedValueIsNew) iTorqueUmotIntegrator += idiffSpeed2;
    }

	iTorqueUmotIntegrator += iTemp1;

	iTorqueProp      = iTemp1 * 8;


	if(iTorqueUmotIntegrator >  iTorqueLimit)  iTorqueUmotIntegrator =  iTorqueLimit;
	if(iTorqueUmotIntegrator < -iTorqueLimit)  iTorqueUmotIntegrator = -iTorqueLimit;

	iTorqueReferenz = (iTorqueUmotIntegrator + iTorqueProp) / defREFERENZFACTOR;
	//-------------------------------------------------


	if(iStep1==0)
	{
		iTorqueLimit 	 		= TORQUE_INTEGRATOR_MAX * 4;
		iFieldIntegrator   		= 0;
		iTorqueUmotIntegrator   = 0;
	}
}



void CalcDiffSpeed()
{
int iTemp1;
int iHysteresis;
int iDiffSpeedMax;

		idiffSpeed2 = 0;
		iTemp1      = iSpeedIntegrator/65536;
    	idiffSpeed1 = iTemp1 - iActualSpeed;
        if(!idiffSpeed1) return;

		if(iTemp1 < 0) iTemp1 = -iTemp1; 					  // absolut value
		iHysteresis = iTemp1 * iParHysteresisPercent;
		iHysteresis /= 100;

		iDiffSpeedMax = iTemp1 * iParDiffSpeedMax;
		iDiffSpeedMax /= 100;

        //--------- calc idiffSpeed2 -----------------
		if(iDiffSpeedMax)
		{
		idiffSpeed2 = idiffSpeed1;
		if(idiffSpeed2 < 0) idiffSpeed2 *= -1;
		idiffSpeed2 *= idiffSpeed2;       // square
		idiffSpeed2 /= iDiffSpeedMax;
		}

		if(idiffSpeed2 < iHysteresis)      idiffSpeed2 /= 4;  			 // Hysteresis
		if(idiffSpeed2 > iDiffSpeedMax)    idiffSpeed2 = iDiffSpeedMax;  // Limit

		if(idiffSpeed1 < 0)idiffSpeed2 *= -1;
}

//======================== SPEED --- CONTROL ====================================
void SpeedControl()
{
		CalcDiffSpeed();

    	if(iSpeedIntegrator < 0) idiffSpeed2 = -idiffSpeed2;  			// invert if speed negativ
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
	iTemp = iSpeedIntegrator/65536;
	if(iTemp < 0) iTemp = -iTemp;

	if(iTemp > iParRpmUmotMax) iTemp = iParRpmUmotMax; // set limit
	iUmotSquare  = iTemp * iTemp;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare *= 4096;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare += iParUmotStart;

	iUmotLinear = iTemp * 4096;
	iUmotLinear /= iParRpmUmotMax;

	iUmotProfile = iUmotLinear;

	if(iUmotSquare > iUmotLinear)  iUmotProfile = iUmotSquare;

	if(iUmotProfile > 4096)        iUmotProfile = 4096;
	//----------------------------------------------------------------


	//--------------- umot limit ---------------------------------

	//================== speed limit =========================================
	/*
	if(iActualSpeed > 0)
	{
	if(iActualSpeed > (defParRpmMotorMax+100))      if(iUmotRpmLimit > iUmotResult) iUmotRpmLimit = iUmotResult;
	if(iActualSpeed < (defParRpmMotorMax-200))      iUmotRpmLimit += 16;
	}
	if(iActualSpeed < 0)
	{
	if(iActualSpeed < (-defParRpmMotorMax-100))     if(iUmotRpmLimit > iUmotResult) iUmotRpmLimit = iUmotResult;
	if(iActualSpeed > (-defParRpmMotorMax+200))     iUmotRpmLimit += 64;
	}
	if(iUmotRpmLimit > UMOT_MAX) iUmotRpmLimit = UMOT_MAX;
    //========================================================================

	if(iUmotResult > iUmotRpmLimit)iUmotResult = iUmotRpmLimit;
    */

	if(iStep1 == 0)
    {
    	iUmotRpmLimit = 4096;
    }
   /*
	iTemp = iActualSpeed;	if(iTemp < 0) iTemp = -iTemp;
	if(iTemp > iParRpmMotorMax) iUmotRpmLimit++;
	else
	{if(iUmotRpmLimit > 0) iUmotRpmLimit--;}

	iUmotResult -=  iUmotRpmLimit;
	*/


}



int CalcUmotProfile()
{
int iTemp;
int iUmotReturn;

//==================== uu  === umot calculation =================
	iTemp = iSpeedIntegrator/65536;
	if(iTemp < 0) iTemp = -iTemp;

	if(iTemp > iParRpmUmotMax) iTemp = iParRpmUmotMax; // set limit
	iUmotSquare  = iTemp * iTemp;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare *= 4096;
	iUmotSquare /= iParRpmUmotMax;
	iUmotSquare += iParUmotStart;

	iUmotLinear = iTemp * 4096;
	iUmotLinear /= iParRpmUmotMax;

	iUmotReturn = iUmotLinear;

	if(iUmotSquare > iUmotLinear)  iUmotReturn = iUmotSquare;

	if(iUmotReturn > 4096)        iUmotReturn = 4096;
	return(iUmotReturn);
}



void RampAccQ1()
{
iSpeedIntegrator += iMotPar[24];
if(iSpeedIntegrator > iSpeedSetUser)  iSpeedIntegrator = iSpeedSetUser;
}

void RampDecQ1()
{
iSpeedIntegrator -= iMotPar[25];
if(iSpeedIntegrator < iSpeedSetUser)  iSpeedIntegrator = iSpeedSetUser;
}

void RampAccQ3()
{
iSpeedIntegrator -= iMotPar[24];
if(iSpeedIntegrator < iSpeedSetUser)  iSpeedIntegrator = iSpeedSetUser;
}

void RampDecQ3()
{
iSpeedIntegrator  += iMotPar[25];
if(iSpeedIntegrator > iSpeedSetUser)  iSpeedIntegrator = iSpeedSetUser;
}




void function_RampPlus()
{
//int iTemp1;
/*
  if(iSpeedSetUser >  iSpeedIntegrator)
  {
	  iTemp1 = (iSpeedIntegrator/65536) - iActualSpeed;
	  if(iTemp1 > 80)
	  {
	  	iSpeedIntegrator -= iMotPar[24];
	  	return;
	  }

	  if(iRampBlocked)   {iRampBlocked = 0;	return;	  }
	  RampAccPlus();
	  return;
  }

  if(iSpeedSetUser <  iSpeedIntegrator)
  {
	  RampDec();
  }
  */
}




//int iTemp1;
//	iTemp1    = (iSpeedIntegrator/65536) - iActualSpeed;
//	if(iTemp1 > 50)return;

void function_Q1()
{
	if(iSpeedSetUser > iSpeedIntegrator) RampAccQ1();
	if(iSpeedSetUser < iSpeedIntegrator) RampDecQ1();
}

void function_Q3()
{
	if(iSpeedSetUser < iSpeedIntegrator) RampAccQ3();
	if(iSpeedSetUser > iSpeedIntegrator) RampDecQ3();
}



void CalcRampForSpeed(){
//int iTemp1;

if(iSpeedSetUser >= 0)
{
	if(iSpeedIntegrator >= 0) { function_Q1(); return; }
	if(iSpeedIntegrator  < 0) { iSpeedIntegrator += iMotPar[25]; } //Q4 go to zero
}

if(iSpeedSetUser <= 0)
{
	if(iSpeedIntegrator <= 0) { function_Q3(); return; }
	if(iSpeedIntegrator  > 0) { iSpeedIntegrator -= iMotPar[25]; } //Q2 go to zero
}

}//end CalcRampForSpeed






int iAngleStart=0;
int iCountGo;
int iEncoderStart;
int iEncoderDiff;
int iEncoderFirst;
int iEncoderSum;


void    function_SensorLessControl()
{
		switch(iStep1)
		{
		case 0: iPwmOnOff 		 	= 0;				// PWM off
				iUmotProfile     	= 0;
				iUmotResult      	= 0;
				iSpeedIntegrator    = 0;
				iMotDirection    	= 0;
				if(iSpeedSetUser != 0) iStep1++;
				iEncoderStart   	= iEncoderAngle;
				iEncoderFirst   	= iEncoderAngle;
				iAngleStart        &= 0x0FFF;
				iAngleSensorLessPWM = iAngleStart;
				iEncoderSum = 0;
				break;

		case 1: iStep1++; 				// start motor
				iCountGo=0;
				if(iSpeedSetUser > 0){iMotDirection =  1; iFieldIntegrator=0; iTorqueUmotIntegrator =  4096;  iTorqueF0 =  500;   }
				if(iSpeedSetUser < 0){iMotDirection = -1; iFieldIntegrator=0; iTorqueUmotIntegrator = -4096;  iTorqueF0 = -500;   }
				iPwmOnOff	= 1;
				break;

		case 2: if(iSpeedSetUser == 0) iStep1=0;     // motor stops from user

				if(iEncoderStart > 2048 && iEncoderAngle < 2048)
				iEncoderDiff = ((iEncoderAngle+4096) - iEncoderStart);
				else
				iEncoderDiff = (iEncoderAngle - iEncoderStart);

				iEncoderSum += iEncoderDiff;
				iEncoderStart = iEncoderAngle;

				if(iCountGo++ > 11000) iStep1++;       // about 3/4 second
				if(iEncoderDiff  < 0){ iStep1=9; iCountGo=0; iAngleStart += 512;  return; }
				iAngleSensorLessPWM += iEncoderDiff;
				iAngleSensorLessPWM &= 0x0FFF;
				break;

		case 3: iCountGo=0;
				if(iEncoderSum > 30) { iEncoderSum=0; iStep1=2;}
				else iStep1=7;
				break;

		case 7: iPwmOnOff 		 = 0;
				iCountGo++;
				if(iCountGo > 9000) iStep1++; // half second
				break;

		case 8: iAngleStart    += 256;
			  	iStep1=0;
			    break;

		case 9: iPwmOnOff 		 = 0;
				iCountGo++;
				if(iCountGo > 9000) iStep1=8; // half second
				break;
//-----------------------------------------------------------------------



//------------------------- overcurrent ----------------------------------
		case 30:  iPwmOnOff		  = 0;			     // error motor stop
				  if(iSpeedIntegrator== 0)iStep1=0;
				  break;

		default:  iStep1 = 0;	  break;
		}

			// pwm 13889 * 4 nsec = 55,556µsec  18Khz
			// electrical: RefPeriod = 4096 * (1/18000)  = 227,56msec RefFreq= 4,394Hz => 263.67RPM
			// motor mechanical: 1000RPM  electrical 7000RPM => 7000/RefRPM = 26,548


			if(iStep1 != 0)
			iUmotResult = (iParUmotBoost + iParUmotStart) * 2;
}// end of funcion_SensorLessControl




#endif

