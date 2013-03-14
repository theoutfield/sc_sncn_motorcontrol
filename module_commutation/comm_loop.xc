#include <xs1.h>
#include <stdint.h>
#include <xscope.h>
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "hall_client.h"
#include "comm_loop.h"
#include "var.h"
#include "def.h"
#include "dc_motor_config.h"

#define DEBUG_commutation

static t_pwm_control pwm_ctrl;

#ifdef DEBUG_commutation
	#include <print.h>
extern out port p_ifm_ext_d1;
extern out port p_ifm_ext_d2;
extern out port p_ifm_shared_leds_wden;  // XS1_PORT_4B; /* BlueGreenRed_Green */
#endif

unsigned char cLeds;
int iHallPositionZero=0;

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue)
{
	unsigned cmd1;
	unsigned cmd2;
	unsigned char cFlag=0;


    //-------------- init values --------------
	InitParameter();
	SetParameterValue();

	 //================== pwmloop ========================
		while (1)
		{
		#ifdef DEBUG_commutation
			cFlag |= 1;
			#ifdef DC900
				p_ifm_ext_d1 <: cFlag;  // set to one
			#endif
		#endif


		//============================== rotor position  from hall ================================================
		{iHallActualSpeed, iHallAngle, iHallPositionAbsolut, iHallPinState}             = get_hall_values(c_hall);

		iHallSpeedValueIsNew   = iHallActualSpeed & 0xFF000000;   		// extract info if SpeedValue is new
		iHallActualSpeed    &= 0x00FFFFFF;
		if(iHallActualSpeed  & 0x00FF0000)
			iHallActualSpeed |= 0xFFFF0000;   						    // expand value if negativ
	    //==========================================================================================================




		//------------------- encoder values-------------------------------------------------------------------------
		{iEncoderActualSpeed, iEncoderAngle, iEncoderPositionAbsolut, iEncoderPinState} = get_encoder_values(c_hall);

		iEncoderAngle *= POLE_PAIRS;
		iEncoderAngle *= 4096;
		iEncoderAngle /= iMotPar[7];  					// encoder steps per resolution

		iEncoderAngle  = iEncoderAngle % 4096;      // modulo value

		if(iActualSpeed >= 0)
		iEncoderAngle -= iMotPar[8]; 					// defParEncoderZeroPoint  0-4095 to syncrinize with hall
		else iEncoderAngle -= 1970;

		iEncoderAngle &= 0x0FFF;

		iEncoderSpeedValueIsNew   = iEncoderActualSpeed & 0xFF000000;   		// extract info if SpeedValue is new
		iEncoderActualSpeed    &= 0x00FFFFFF;
		if(iEncoderActualSpeed  & 0x00FF0000)
			iEncoderActualSpeed |= 0xFFFF0000;   						    // expand value if negativ

		//-----------------------------------------------------------------------------------------------------------


		if(iEncoderOnOff==0)  // 0=Hall 1=Encoder
		{
		iAngleRotor      = iHallAngle & 0x0FFF;
		iActualSpeed     = iHallActualSpeed;
		iSpeedValueIsNew = iHallSpeedValueIsNew;
		iPositionAbsolut = iHallPositionAbsolut;
		}
		else
		{
		iAngleRotor  	 = iEncoderAngle & 0x0FFF;
		iActualSpeed 	 = iEncoderActualSpeed;
		iSpeedValueIsNew = iEncoderSpeedValueIsNew;
		iPositionAbsolut = iEncoderPositionAbsolut;
		}



		iDiffAngleRotor = iAngleRotor - iAngleRotorOld;
		if(iActualSpeed > 0)
		{
			if(iAngleRotorOld > 2048  && iAngleRotor < 2048)
			cTriggerPeriod = 0xFF;
		if(iDiffAngleRotor < 0) iDiffAngleRotor += 4096;
		}
		if(iActualSpeed < 0)
		{
			if(iAngleRotorOld < 2048  && iAngleRotor > 2048)
			cTriggerPeriod = 0x7F;
			if(iDiffAngleRotor > 0) iDiffAngleRotor -= 4096;
		}

		// electrical: RefPeriod = 4096 * (1/18000)  = 227,56msec RefFreq= 4,394Hz => 263.67RPM
		// motor mechanical: 1000RPM  electrical 7000RPM => 7000/RefRPM = 26,548
		// gear 1:156    500RPM ca.20sec one rotation


        if(iSpeedValueIsNew){
		iAngleRotorDiffCalculated  = iActualSpeed * 700;
		iAngleRotorDiffCalculated /= 26367;
        }



		iAngleRotorOld = iAngleRotor;


		//===================================================================================

		CalcCurrentValues();

		//***************** steps ***********************************************************

		switch(iControlFOC)
		{
		case 0:
		case 1:  function_SpeedControl();
		         break;
		case 2:  function_TorqueControl();
		         break;
		case 3:  function_PositionControl();
		         break;
		case 4:  function_SensorLessControl();
		         break;
		default: iControlFOC=0; break;
		}

		iLoopCount++;
		iLoopCount &= 0x03;
		switch(iLoopCount)
		{
			case 0:	iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
					iVectorInvPark = root_function(iVectorInvPark);
					break;

			case 1:	iVectorCurrent = iAlpha * iAlpha + iBeta * iBeta;
					iVectorCurrent = root_function(iVectorCurrent);
					break;

			case 2: if(a1SquareMean)
						a1RMS = root_function(a1SquareMean);
						a1SquareMean = 0;
					break;

			case 3: if(a2SquareMean)
						a2RMS = root_function(a2SquareMean);
						a2SquareMean = 0;
					break;
		}


    //********************************************************************

		iPowerMotor = 6863 * iIqPeriod2;
		iPowerMotor /= 65536;
		iPowerMotor *= iActualSpeed;
		iPowerMotor /= 4944;


		//===========  if a1RMS > Limit block Umot and ramp ==============
		if(a1RMS >= iParRMS_RampLimit)
		{
			if(iUmotResult > iUmotLast) iUmotResult = iUmotLast;
			iRampBlocked = 1;
		}


		if(iActualSpeed >= iParRpmMotorMax)  // Limit Max Speed  +/- vvvvv
		{
			if(iUmotResult > iUmotLast) iUmotResult = iUmotLast;
		}



		//=========== calculate iAngleDiffPeriod  only for view ============================
	#ifdef DEBUG_commutation
		iAngleDiff = 0;
		if(iActualSpeed > 0)
		{
			iAngleDiff = iAngleCurrent - iAngleRotor;
			if(iAngleDiff < 0)iAngleDiff += 4096;
		}
		if(iActualSpeed < 0)
		{
			iAngleDiff = iAngleRotor - iAngleCurrent;
			if(iAngleDiff < 0)iAngleDiff += 4096;
		}
		iAngleDiffSum += iAngleDiff;
		iCountAngle++;
		if(cTriggerPeriod & 0x04 || iCountAngle > 36000)
		{
			cTriggerPeriod &= 0x04^0xFF;
			if(iCountAngle){
				iAngleDiffPeriod = iAngleDiffSum/iCountAngle;
				iAngleDiffSum    = 0;
				iCountAngle      = 0;
			}
		}//end if(cTriggerPeriod
	#endif

	//============================================================================================

	//============================= set angle for pwm ============================================
		if (iMotDirection !=  0)
		{
			iAnglePWMFromFOC  = iAngleInvPark;
			if(iActualSpeed >= 0)  iAnglePWMFromFOC   +=  (3076 + iParAngleUser);
			                  else iAnglePWMFromFOC   +=  (3076 + iParAngleUser) +120;
			iAnglePWMFromFOC  &= 0x0FFF;
			iAnglePWM = iAnglePWMFromFOC;
		}
		iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff

		if(iStep1 == 0)
		{
			iUmotIntegrator = 0;
			iUmotP			= 0;
			iUmotResult     = 0;
			iIqPeriod2		= 0;
			iIdPeriod2		= 0;
		}

	   //================== Holding Torque if motor stopped ====================================================
        if(iUmotResult < iMotHoldingTorque)iUmotResult = iMotHoldingTorque;

		if(iMotHoldingTorque)
		{
			if(iStep1 != 0) iAngleLast = iAnglePWM;
			if(iStep1 == 0 )
			iAnglePWM = iAngleLast; //  + iAngleRotor  - 600;
		}

		//======================================================================================================
		//======================================================================================================



		//------------ iUmotMotor follows iUmotResult -----------
		iUmotLast = iUmotResult;
		if(iUmotResult > iUmotMotor) iUmotMotor++;
		if(iUmotResult < iUmotMotor) iUmotMotor--;
		p_ifm_ext_d1 <: 0; // yellow


	#ifdef DEBUG_commutation
/*
	 	 xscope_probe_data(0,iPhase1);
	 	 xscope_probe_data(1,a1RMS);
	 	 xscope_probe_data(2,iSetLoopSpeed);
	 	 xscope_probe_data(3,iAngleRotor);
	 	 xscope_probe_data(4,iAngleCurrent);
	   	 xscope_probe_data(5,iUmotMotor);
	   	 xscope_probe_data(6,a1Square/1000);
*/
/*		 xscope_probe_data(0,iActualSpeed);
		 xscope_probe_data(1,iSetLoopSpeed);
		 xscope_probe_data(2,iUmotIntegrator/256);
		 xscope_probe_data(3,iUmotMotor);
		 xscope_probe_data(4,iAngleDiffPeriod);
		 xscope_probe_data(5,iVectorInvPark);
		 xscope_probe_data(6,iVectorCurrent);
		 xscope_probe_data(7,iIdPeriod2);
		 xscope_probe_data(8,iIqPeriod2);

*/		 xscope_probe_data(0,iPhase1);
		 xscope_probe_data(1,iAngleCurrent);
		 xscope_probe_data(2,iAnglePWM);
		 xscope_probe_data(3,iAngleRotor);
		 xscope_probe_data(4,iEncoderAngle);
		 xscope_probe_data(5,iAnglePWMFromHall);
		 xscope_probe_data(6,iAnglePWMFromFOC);
		 xscope_probe_data(7,iVectorCurrent);
		 xscope_probe_data(8,iVectorInvPark);
		 xscope_probe_data(9,iDiffAngleRotor);

	#endif

        if(iControlFOC == 4) iAnglePWM = iAngleSensorLessPWM;
		iIndexPWM = iAnglePWM >> 2;  // from 0-4095 to LUT 0-1023


		//======================== read current ============================
				#ifdef DC900
				{a1,a2,adc_a1,adc_a2,adc_a3, adc_a4,adc_b1,adc_b2,adc_b3,adc_b4}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
				#endif
				 a1 = -a1;
				 a2 = -a2;
				// a3 = -a1 -a2;

				#ifdef DEBUG_commutation
					p_ifm_ext_d2 <: 1;
				#endif



				#ifdef DC100
				 {a1, a2, a3}  = get_adc_vals_calibrated_int16_ltc1408( adc );
					#ifdef DEBUG_commutation
				 	 cFlag |= 0x02; testport <: cFlag;  // oszi green C4
					#endif
				#endif

				#ifdef DEBUG_commutation
					#ifdef DC900
				 	 	 p_ifm_ext_d2 <: 0;
					#endif
				#endif
//======================== end current ===============================================0000



		sine_pwm( iIndexPWM, iUmotMotor, iMotHoldingTorque , pwm_ctrl, c_pwm_ctrl, iPwmOnOff );



		#ifdef DEBUG_commutation
			cLeds = 0;
			#define defpp1  0
			#define defpp2  64
			#define defpp3  128
			#define defpp4  192

			if(iIndexPWM > defpp1   && iIndexPWM < (defpp1+8)) cLeds = 0x01;
			if(iIndexPWM > defpp2   && iIndexPWM < (defpp2+8)) cLeds = 0x02;
			if(iIndexPWM > defpp3   && iIndexPWM < (defpp3+8)) cLeds = 0x04;
			if(iIndexPWM > defpp4   && iIndexPWM < (defpp4+8)) cLeds = 0x08;
			cLeds ^= 0xFF;
			p_ifm_shared_leds_wden <: cLeds;
		#endif


		SaveValueToArray();

		//================== uart connection with one pin =============================================
		select
		{
		case c_motvalue :> cmd2:
		    	if(cmd2 >= 0 && cmd2 <= 12)
					{
					c_motvalue :> iMotCommand[cmd2];
					c_motvalue :> iMotCommand[cmd2+1];
					c_motvalue :> iMotCommand[cmd2+2];
					c_motvalue :> iMotCommand[cmd2+3];
					}
			 	else if(cmd2 >= 32 && cmd2 < 64)
						{  	c_motvalue <: iMotValue[cmd2-32];
							c_motvalue <: iMotValue[cmd2-31];
							c_motvalue <: iMotValue[cmd2-30];
							c_motvalue <: iMotValue[cmd2-29];
					    }

				else if(cmd2 >= 64 && cmd2 < 96)  { c_motvalue <: iMotPar[cmd2-64]; 	}
				else if(cmd2 >= 96 && cmd2 < 128) { c_motvalue :> iMotPar[cmd2-96]; iUpdateFlag=1;}
				break;
				default:	break;
		}// end select

		//=================================================================================

		select
		{
			case c_commutation :> cmd1:
		    	if(cmd1 >= 0 && cmd1 <= 12)
					{
		    		c_commutation :> iMotCommand[cmd1];
		    		c_commutation :> iMotCommand[cmd1+1];
		    		c_commutation :> iMotCommand[cmd1+2];
		    		c_commutation :> iMotCommand[cmd1+3];
					}
				else if(cmd1 >= 32 && cmd1 < 64)  {  c_commutation <: iMotValue[cmd1-32]; 	}
				else if(cmd1 >= 64 && cmd1 < 96)  {  c_commutation <: iMotPar[cmd1-64]; 	}
				else if(cmd1 >= 96 && cmd1 < 128) {  c_commutation :> iMotPar[cmd1-96]; iUpdateFlag=1;}
				break;
			default: break;
		}// end select
//--------------------------------------------------------------------------
		 if(iMotCommand[7]==1)
		 {
		  iMotCommand[7] = 0;

			CalcSetInternSpeed(iMotCommand[0]);
			iControlFOC 			= iMotCommand[1] & 0x07;
			iEncoderOnOff           = iMotCommand[1] & 0x08;
			iTorqueSet  		    = iMotCommand[2];
			iMotHoldingTorque       = iMotCommand[3];
			iPositionAbsolutNew     = iMotCommand[5];
		}

		if(iUpdateFlag)	{ iUpdateFlag=0; SetParameterValue(); }

	}// end while(1)

}// end function






void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  do_adc_calibration_ad7949(c_adc);
	  comm_sine(c_adc, c_commutation, c_hall, c_pwm_ctrl, c_motvalue);
}
//===============================  utilities ===================================

void CalcSetInternSpeed(int iSpeedValue)
{
    iSetValueSpeed = iSpeedValue;

	if(iSetValueSpeed > iParRpmMotorMax)	iSetValueSpeed = iParRpmMotorMax;
	iSetInternSpeed = iSetValueSpeed * 65536;
}









