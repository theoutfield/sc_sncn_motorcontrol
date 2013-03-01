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

#define DEBUG_commutation

static t_pwm_control pwm_ctrl;

#ifdef DEBUG_commutation
	#include <print.h>
extern out port p_ifm_ext_d1;
extern out port p_ifm_ext_d2;
extern out port p_ifm_shared_leds_wden;  // XS1_PORT_4B; /* BlueGreenRed_Green */
#endif

unsigned char cLeds;
int iPositionZero=0;

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl, chanend c_motvalue)
{
	unsigned cmd1;
	unsigned cmd2;
	int iTemp,iTemp1=0;
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

		//============= rotor position  from hall ===============================
		iLoopCount++;
		iLoopCount &= 0x0F;
		switch(iLoopCount & 0x03)
		{
			case 0: iPositionAbsolut  = get_hall_absolute_pos(c_hall);
			        iPositionAbsolut -= iPositionZero;
					break;
			case 1: iPinStateHall    = get_hall_pinstate(c_hall);
					break;
			case 2: iPinStateEncoder = get_encoder_pinstate(c_hall);
					break;
			case 3: iPositionEncoder = get_encoder_position(c_hall);
					break;
		}


		iActualSpeed     =  get_hall_speed(c_hall);
		iAngleFromHall   = 	get_hall_angle(c_hall);
		iAngleFromHall  &=  0x0FFF;

		iSpeedValueNew   = iActualSpeed & 0xFF000000;   				// extract info if SpeedValue is new
		iActualSpeed    &= 0x00FFFFFF;
		if(iActualSpeed & 0x00FF0000)
			iActualSpeed |= 0xFFFF0000;   							    // expand value if negativ


		if(iActualSpeed > 0)
		{
			if(iAngleFromHallOld > 2048  && iAngleFromHall < 2048)
			cTriggerPeriod = 0xFF;
		}
		if(iActualSpeed < 0)
		{
			if(iAngleFromHallOld < 2048  && iAngleFromHall > 2048)
			cTriggerPeriod = 0x7F;
		}


		if(iAngleFromHall != iAngleFromHallOld)
		{
			if(iAngleFromHall > iAngleFromHallOld)
				iDiffAngleHall = iAngleFromHall - iAngleFromHallOld;
			else
				iDiffAngleHall = (iAngleFromHall +4096) - iAngleFromHallOld;
		}

		iAngleFromHallOld = iAngleFromHall;


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
		default: iControlFOC=0; break;
		}



		switch(iLoopCount & 0x03)
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
			iAngleDiff = iAngleCurrent - iAngleFromHall;
			if(iAngleDiff < 0)iAngleDiff += 4096;
		}
		if(iActualSpeed < 0)
		{
			iAngleDiff = iAngleFromHall - iAngleCurrent;
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
			iAnglePWMFromHall = iAngleFromHall + iParAngleUser ;
			iAnglePWMFromHall &= 0x0FFF;
			iAnglePWMFromFOC  = iAngleInvPark + (4096 - 1020) + iParAngleUser;
			iAnglePWMFromFOC  &= 0x0FFF;
			iAngleXXX = iAnglePWMFromFOC - iAnglePWMFromHall;
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
			iAnglePWM = iAngleLast; //  + iAngleFromHall  - 600;
		}

		//======================================================================================================
		//======================================================================================================



		//------------ iUmotMotor follows iUmotResult -----------
		iUmotLast = iUmotResult;
		if(iUmotResult > iUmotMotor) iUmotMotor++;
		if(iUmotResult < iUmotMotor) iUmotMotor--;



	#ifdef DEBUG_commutation
/*
	 	 xscope_probe_data(0,iPhase1);
	 	 xscope_probe_data(1,a1RMS);
	 	 xscope_probe_data(2,iSetLoopSpeed);
	 	 xscope_probe_data(3,iAngleFromHall);
	 	 xscope_probe_data(4,iAngleCurrent);
	   	 xscope_probe_data(5,iUmotMotor);
	   	 xscope_probe_data(6,a1Square/1000);
*/
		 xscope_probe_data(0,iActualSpeed);
		 xscope_probe_data(1,iSetLoopSpeed);
		 xscope_probe_data(2,iUmotIntegrator/256);
		 xscope_probe_data(3,iUmotMotor);
		 xscope_probe_data(4,iAngleDiffPeriod);
		 xscope_probe_data(5,iVectorInvPark);
		 xscope_probe_data(6,iVectorCurrent);
		 xscope_probe_data(7,iIdPeriod2);
		 xscope_probe_data(8,iIqPeriod2);

/*		 xscope_probe_data(0,iPhase1);
		 xscope_probe_data(1,iAngleCurrent);
		 xscope_probe_data(2,iAnglePWM);
		 xscope_probe_data(3,iAngleFromHall);
		 xscope_probe_data(4,iAngleInvPark);
		 xscope_probe_data(5,iAnglePWMFromHall);
		 xscope_probe_data(6,iAnglePWMFromFOC);
		 xscope_probe_data(7,iVectorCurrent);
		 xscope_probe_data(8,iVectorInvPark);
		 xscope_probe_data(9,iPhase2);
*/
	#endif
		// pwm 13889 * 4 nsec = 55,556µsec  18Khz
/*
		if(iControlFOC==xxx){
		iPwmIndexHigh += iPwmAddValue;
		iPwmIndexHigh &= 0x0FFFFFFF;
		iAnglePWM	   = iPwmIndexHigh/65536;
		}
*/

		iIndexPWM = iAnglePWM >> 2;  // from 0-4095 to LUT 0-1023
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

			p_ifm_ext_d1 <: 0; // yellow
		#endif


	//======================== read current ============================
		#ifdef DC900
		{a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
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

		SaveValueToArray();


		//================== uart connection with one pin ========================
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
				else if(cmd2 >= 96 && cmd2 < 128) { c_motvalue :> iMotPar[iTemp-96]; iUpdateFlag=1;}
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
				else if(cmd1 >= 32 && cmd1 < 64)  { iTemp = (int) cmd1; c_commutation <: iMotValue[iTemp-32]; 	}
				else if(cmd1 >= 64 && cmd1 < 96)  { iTemp = (int) cmd1; c_commutation <: iMotPar[iTemp-64]; 	}
				else if(cmd1 >= 96 && cmd1 < 128) { iTemp = (int) cmd1; c_commutation :> iTemp1;  iMotPar[iTemp-96] = iTemp1; iUpdateFlag=1;}
				break;
			default: break;
		}// end select
//--------------------------------------------------------------------------
		 if(iMotCommand[15]==1)
		 {
		  iMotCommand[15] = 0;

			CalcSetInternSpeed(iMotCommand[0]);
			if(iControlFOC != iMotCommand[1])
			{
			iStep1 = 0;
			iControlFOC = iMotCommand[1];
			}
			iTorqueSet  		    = iMotCommand[6];
			iMotHoldingTorque       = iMotCommand[7];
			iPositionAbsolutNew     = iMotCommand[10];
			iPositionZero           = iMotCommand[11];
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
	if(iParDefSpeedMax > 0)
	{
		iSetValueSpeed *= iParRPMreference;
		iSetValueSpeed /= iParDefSpeedMax;
	}
	if(iSetValueSpeed > iParRpmMotorMax)	iSetValueSpeed = iParRpmMotorMax;
	iSetInternSpeed = iSetValueSpeed * 65536;
}









