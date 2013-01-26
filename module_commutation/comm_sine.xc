#include <xs1.h>
#include <stdint.h>
#include <print.h>
#include <xscope.h>
#include "pwm_cli_inv.h"
#include "sine_lookup.h"
#include "refclk.h"
#include "predriver/a4935.h"
#include "adc_client_ad7949.h"
#include "adc_client_ltc1408.h"
#include "hall_client.h"
#include "sine_lookup.h"

#define DC900

static t_pwm_control pwm_ctrl;

extern short sine_third[];
extern short arctg_table[];

#ifdef DC100
extern out port testport;
#endif

#ifdef DC900
extern out port p_ifm_ext_d0;
extern out port p_ifm_ext_d1;
extern out port p_ifm_ext_d2;
#endif



unsigned root_function(unsigned uSquareValue);

void comm_sine_init(chanend c_pwm_ctrl)
{
	unsigned pwm[3] = { PWM_MAX_VALUE/2, PWM_MAX_VALUE/2, PWM_MAX_VALUE/2 };  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}



#define defRampMax 8192*2			//ramp params
#define defRampMin 32

void comm_sine(chanend adc, chanend c_commutation, chanend c_hall, chanend c_pwm_ctrl)
{
	unsigned cmd;
	unsigned pwm[3] = { 0, 0, 0 };
	int iTemp;
	int iCountx;
	unsigned char cFlag=0;

	int iStep1=0;
	int iStepRamp=0;

	int iUmotSquare, iUmotLinear, iUmotSocket, iUmotBoost;
	int iUmotIntegrator=0 , iUmotP, iUmotResult = 0, iUmotMotor = 0;   // follows iUmotResult
	int iIndexPWM, iPosFromHallOld=0;
	int iAngleFromHall  = 0;
	int iAngleUser      = 300;
	int iAngleFromRpm   = 0;
	int iAnglePWM;

	int iPositionAbsolut=0;

	int iCountRMS=0;
	int a1Square,a2Square,a3Square;
	int a1RMS,a2RMS,a3RMS;
	int a1, a2, a3;
	int iAlpha,iBeta;
	int iAngleCur;
	int iPhase1=0;
	int iPhase2=0;
	int iPhase3=0;
	int iPhase1Sum=0;
	int iPhase2Sum=0;
	int iPhase3Sum=0;
	int iId,iIdPeriod;
	int iIq,iIqPeriod;
	int iIdPeriod2=0;
	int iIqPeriod2=0;
	int iCountDivFactor;


	int sinx,cosx;
	unsigned theta;  // angle

	unsigned uCurVector;

	int iRampAccValue=16;
	int iRampDecValue=16;
	int iRampIntegrator;

	int iSetValueSpeed=0;
	int iSetInternSpeed=0;
	int iSetSpeedRamp=0;

	int iSetSpeed=0;
	int iActualSpeed=0;
	int idiffSpeed;
	int idiffSpeed2;  // idiffSpeed with hyteresis
	char cTriggerPeriod=0;

	int iPwmOnOff = 1;
	//-------------- init values --------------
	iUmotSocket = 150;
	a1RMS = 0;
	a2RMS = 0;
	a3RMS = 0;
	iCountRMS=0;

	#define defRMSLimit 1200



	while (1)
	{
		p_ifm_ext_d0 <: 1;  // set to one

		//============= rotor position ===============================
		iPositionAbsolut = get_hall_absolute_pos(c_hall);
		iActualSpeed = get_hall_speed(c_hall);
		iAngleFromHall = get_hall_angle(c_hall);

		 if(iActualSpeed > 0)
		{
		if(iPosFromHallOld > 2048  && iAngleFromHall < 2048) cTriggerPeriod=0xFF;    //test
		}
		if(iActualSpeed < 0)
		{
		if(iPosFromHallOld < 2048  && iAngleFromHall > 2048) cTriggerPeriod=0x7F; 	//test
		}
		iPosFromHallOld = iAngleFromHall;

	 //============== calc Umot ============================
		#define defMaxValue 3000		// controller
		#define defInt_factor  4
		#define defProp_factor 4

		 iTemp = iSetSpeed;
		 if(iTemp < 0) iTemp = -iTemp;

		 iUmotSquare = iTemp * iTemp / defMaxValue;
		 iUmotSquare *= 4096;
		 iUmotSquare /= defMaxValue;
		 iUmotSquare += iUmotSocket;

		 iUmotLinear = iTemp * 4096;
		 iUmotLinear /= defMaxValue;
		 iUmotResult = iUmotLinear;

		 if(iUmotSquare > iUmotLinear) iUmotResult = iUmotSquare;
	//=========================================================


	//***************** Steps****************************************

			 switch(iStep1)	//controller steps
		{
			case 0: a1=0; a2=0; a3=0;
					iPwmOnOff = 0;
					if(iSetInternSpeed > 0){iUmotBoost = 250 * 32; iStep1= 1; }
					if(iSetInternSpeed < 0){iUmotBoost = 250 * 32; iStep1=21; }
					iUmotResult  = 0;
					iStepRamp    = 1;
					break;

			case 1:  iStepRamp       = 1;
					 iPwmOnOff		 = 1;
					 iRampIntegrator = 0;
					 iUmotMotor = iUmotResult;
					 iStep1++;
					 break;

			case 2:  iUmotMotor = iUmotResult;
					 iStep1++; break;
			case 3:  iStep1++; break;
			case 4:  iStep1++; break;

			case 5:  if(iUmotBoost > 0)iUmotBoost--;
					 if(iUmotBoost > 0)iUmotBoost--;
					 if((iSetSpeedRamp + iRampIntegrator) >= iSetInternSpeed){
					 if(iRampAccValue > defRampMin) iRampAccValue-=8;
					 }
					 if(iSetSpeedRamp == iSetInternSpeed) iStep1=10;
					 if(iSetSpeedRamp  > iSetInternSpeed) iStep1=21;
					 break;

			case 10: iRampAccValue = defRampMin;
					 iRampDecValue = defRampMin;
					 iStepRamp     = 1;
					 if(iSetSpeedRamp  < iSetInternSpeed) iStep1=1;   // acc
					 if(iSetSpeedRamp  > iSetInternSpeed) iStep1=21;  // dec
					 if(iSetInternSpeed==0)iStep1++;
					 iCountx =0;
					 break;

			case 11: if(iCountx++ > 20000) iStep1=0; break;

			case 15: if(iSetSpeedRamp==0)iStep1=0;     // motor is stopping
					 break;

			//---------------------------------------------------

			case 21:  iStepRamp       = 1;
					  iPwmOnOff		  = 1;
					  iRampIntegrator = 0;
					  iUmotMotor = iUmotResult;
					  iRampAccValue =  defRampMin;
					  iStep1++;
					  break;
			case 22:  iStep1++; break;
			case 23:  iStep1++; break;
			case 24:  iStep1++; break;
			case 25:  if(iUmotBoost > 0)iUmotBoost--;
					  if(iUmotBoost > 0)iUmotBoost--;
					  if(iSetSpeedRamp == iSetInternSpeed) iStep1=10;
					  if(iSetSpeedRamp  < iSetInternSpeed) iStep1=1;
					  break;

			case 30:  iPwmOnOff		  = 0;			// error motor stop
					  iStep1++;
					  break;

			case 31:  iPwmOnOff		  = 0;
					  if(iSetSpeed==0)iStep1=0;     // motor is stopping
					  break;

			default: printstr("error\n"); iStep1 = 0; break;
		}

		  //============================== ramp calculation ============
		 if(iSetSpeedRamp < iSetInternSpeed)  // acc == limit +
		 {
			 iRampDecValue = defRampMin;
			 iSetSpeedRamp += iRampAccValue;

			 if(iStepRamp==1)
			 {
				 if(iRampAccValue >= defRampMax)
				 { iStepRamp++; iRampAccValue = defRampMax;}
				 else
				 {
					 iRampIntegrator += iRampAccValue;
					 iRampAccValue += 8;
				 }
			 }
			 if(iSetSpeedRamp > iSetInternSpeed)iSetSpeedRamp = iSetInternSpeed;
		 }

		 if(iSetSpeedRamp > iSetInternSpeed)
		 {
			 iRampAccValue = defRampMin;
			 iSetSpeedRamp -= iRampDecValue;
			 iRampDecValue += 8;
			 if(iRampDecValue >= defRampMax)iRampDecValue = defRampMax;
			 if(iSetSpeedRamp < iSetInternSpeed)iSetSpeedRamp = iSetInternSpeed;
		 }
		 iSetSpeed = iSetSpeedRamp/ 65536;


	 //======== current low pass filter =============================
	 // 66mV/A    16384 = 4.096Volt   2,5V = 16384/4,096 * 2,5 = 10000  16384/4096 * 100 = 400 bits/Ampere
	 iPhase1Sum -= iPhase1;
	 iPhase1Sum += a1;
	 iPhase1 = iPhase1Sum/4;

	 iPhase2Sum -= iPhase2;
	 iPhase2Sum += a2;
	 iPhase2 = iPhase2Sum/4;

	 iPhase3Sum -= iPhase3;
	 iPhase3Sum += a3;
	 iPhase3 = iPhase3Sum/4;

	 //a1Square += (iPhase1 * iPhase1);
	 //a2Square += (iPhase2 * iPhase2);
	 //a2Square += (iPhase3 * iPhase3);

	 a1Square += (a1 * a1);
	 a2Square += (a2 * a2);
	 a3Square += (a3 * a3);

	 if(iStep1==0)
	 {
		 a1Square=0;
		 a2Square=0;
		 a3Square=0;
		 iCountRMS = 0;
	 }
	 iCountRMS++;

	 iTemp = a1;
	 if(iTemp < 0) iTemp = -iTemp;
	 if(iTemp > 3000) iStep1=30;  // Motor stop

	 if(a1RMS > defRMSLimit) iStep1=30;  // Motor stop
	 if(a2RMS > defRMSLimit) iStep1=30;  // Motor stop
	 if(a3RMS > defRMSLimit) iStep1=30;  // Motor stop

	 //=========== RMS ===============
	 if(cTriggerPeriod & 0x02  || iCountRMS > 20000)  // timeout about 100msec
	 {
		 cTriggerPeriod &= 0x02^0xFF;

		 if(iCountRMS){
		 a1RMS = a1Square/iCountRMS;
		 a2RMS = a2Square/iCountRMS;
		 a3RMS = a3Square/iCountRMS;
		 }

		 a1RMS = root_function(a1RMS);
		 a2RMS = root_function(a2RMS);
		 a3RMS = root_function(a3RMS);

		 a1Square  = 0;
		 a2Square  = 0;
		 a3Square  = 0;
		 iCountRMS = 0;
	 }


	 //============== clarke transformation ===================
	 iAlpha = iPhase1;
	 iBeta  = (iPhase1 + 2*iPhase2);   // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
	 iBeta *= 37838;
	 iBeta /= 65536;
	 iBeta = -iBeta;

	 //================== angle = arctg(iBeta/iAlpha) =========
	 iAngleCur  = arctg1(iAlpha,iBeta);
	 uCurVector = iAlpha * iAlpha + iBeta * iBeta;

	 //============= park transform ==qq=========================
	theta = iAngleFromHall/16;
	theta &= 0xFF;
	sinx  = sine_table[theta];      // sine( theta );
	theta = (64 - theta);  		    // 90-theta
	theta &= 0xFF;
	cosx  = sine_table[theta];      // values from 0 to +/- 16384
	iId = (((iAlpha * cosx ) /16384) + ((iBeta * sinx ) /16384));
	iIq = (((iBeta * cosx ) /16384) - ((iAlpha * sinx ) /16384));

	iIdPeriod += iId;
	iIqPeriod += iIq;
	iCountDivFactor++;

	 if(cTriggerPeriod & 0x01)
	 {
		  cTriggerPeriod &= 0x01^0xFF;
		  iIqPeriod2 = iIqPeriod/iCountDivFactor;
		  iIdPeriod2 = iIdPeriod/iCountDivFactor;
		  iIqPeriod  = 0;
		  iIdPeriod  = 0;
		  iCountDivFactor=0;
	 }


	//================ speed-control ==========================
	idiffSpeed = iSetSpeed - iActualSpeed;
	idiffSpeed2 = idiffSpeed;

	if(idiffSpeed > 0) if(idiffSpeed < 5)  idiffSpeed2=0;  // Hysteresis
	if(idiffSpeed < 0) if(idiffSpeed > -5) idiffSpeed2=0;
	if(iSetSpeed < 0) idiffSpeed2 = -idiffSpeed2;

	iUmotIntegrator += idiffSpeed2/defInt_factor;
	iUmotP           = idiffSpeed2/defProp_factor;


	//-------------- set limits -------------------------------
	#define defUmotIntegratorLimit 65536*2

	if(iUmotIntegrator > 0)
	if(iUmotIntegrator > defUmotIntegratorLimit)iUmotIntegrator = defUmotIntegratorLimit;

	if(iUmotIntegrator < 0)
	if(iUmotIntegrator < -defUmotIntegratorLimit)iUmotIntegrator = -defUmotIntegratorLimit;
	//-------------------------------------------------------------

	 iUmotResult += iUmotIntegrator/256;
	 iUmotResult += iUmotP;
	 iUmotResult += (iUmotBoost / 32);

	 if(iUmotResult > 4096)   iUmotResult 	= 4096;
	 if(iUmotResult < 0 )     iUmotResult 	= 0;

	//========== iUmotMotor follows iUmotResult  =================  //addi ramp
	if(iUmotResult > iUmotMotor) iUmotMotor++; // acc
	if(iUmotResult < iUmotMotor) iUmotMotor--; // dec



	iAngleFromRpm = iActualSpeed;
	if(iAngleFromRpm < 0)iAngleFromRpm = -iAngleFromRpm;  // absolut value
	iAngleFromRpm *= 150;
	iAngleFromRpm /= 4000;

	//iAngleFromRpm = 0;

	if (iSetSpeed >= 0)
	{
			iAnglePWM = iAngleFromHall + iAngleUser + iAngleFromRpm  + 300;
	}


	if (iSetSpeed <  0)
	{
			iAnglePWM = iAngleFromHall + iAngleUser - iAngleFromRpm + 2500;
	}

	iAnglePWM &= 0x0FFF; // 0 - 4095  -> 0x0000 - 0x0fff

	//==============================================================================
	if(iStep1 == 0){ iUmotResult = 0; iUmotMotor=0; iUmotIntegrator = 0; iUmotP=0; iIqPeriod2=0; iIdPeriod2=0;
					 a1RMS=0; a2RMS=0; a3RMS=0;
	}


			 xscope_probe_data(0,cTriggerPeriod);
			 xscope_probe_data(1,iPhase2);
			 xscope_probe_data(2,uCurVector);
			 xscope_probe_data(3,iAngleCur);

			//xscope_probe_data(0,iAngleFromHall);
			 //xscope_probe_data(1,iActualSpeed);
			 //xscope_probe_data(0, iPositionAbsolut);
			/* xscope_probe_data(6,iIqPeriod2);
*/
			// xscope_probe_data(0,iSetSpeed);
			// xscope_probe_data(0,iActualSpeed);
			// xscope_probe_data(2,iUmotMotor);
			// xscope_probe_data(3,iIdPeriod2);
			// xscope_probe_data(4,iIqPeriod2);
		//xscope_probe_data(0,iAngleFromHall);

		/* xscope_probe_data(0,a1);
		 xscope_probe_data(1,a2);
		 xscope_probe_data(2,a3);
		 xscope_probe_data(3,iAngleFromHall);
		 xscope_probe_data(4,iAngleCur);*/
	//	   	 xscope_probe_data(5,iId);
	//	   	 xscope_probe_data(6,iIq);

		//commutation routine

		iIndexPWM = iAnglePWM >> 4;
		pwm[0] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM +85) & 0xff;
		pwm[1] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;
		iIndexPWM = (iIndexPWM + 86) & 0xff;
		pwm[2] = ((sine_third[iIndexPWM])*iUmotMotor)/4096   + PWM_MAX_VALUE/2;

		if(pwm[0] < PWM_MIN_LIMIT)      pwm[0] = 0;
		if(pwm[1] < PWM_MIN_LIMIT)      pwm[1] = 0;
		if(pwm[2] < PWM_MIN_LIMIT)      pwm[2] = 0;


	#ifdef DC900
		if(sine_third[iIndexPWM] > 0) p_ifm_ext_d1 <: 1;
		else p_ifm_ext_d1 <: 0;
		p_ifm_ext_d0 <: 0; // yellow
		 {a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
		 a1 = -a1;
		 a2 = -a2;
		 a3 = -a1 -a2;
		p_ifm_ext_d2 <: 1;
	#endif


	 //======================== read current ============================
	 //  register ia ib
	#ifdef DC100
	 {a1, a2, a3}  = get_adc_vals_calibrated_int16_ltc1408( adc );
	 cFlag |= 0x02; testport <: cFlag;  // oszi green C4
	#endif


	 // PWM_MAX_VALUE/2
	 if(iPwmOnOff==0)
	 { pwm[0]=PWM_MAX_VALUE/2;   pwm[1]=PWM_MAX_VALUE/2;   pwm[2]=PWM_MAX_VALUE/2;  }


	  update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

	#ifdef DC900
	  p_ifm_ext_d2 <: 0;
	#endif

	select
	{
		case c_commutation :> cmd:
		  if(cmd == 1) { c_commutation :> iPwmOnOff;     }
		  else if(cmd == 4) { c_commutation :> iSetValueSpeed;   iSetInternSpeed = iSetValueSpeed * 65536;  } //
		  else if(cmd == 5) { c_commutation :> iAngleUser;	}

		  else if(cmd ==  8) { c_commutation <: a1RMS; 	}
		  else if(cmd ==  9) { c_commutation <: a2RMS;  }
		  else if(cmd == 10) { c_commutation <: a3RMS; 	}

		  else if(cmd == 11) { c_commutation <: iUmotSquare;  	}
		  else if(cmd == 12) { c_commutation <: iUmotLinear;  	}
		  else if(cmd == 13) { c_commutation <: iUmotIntegrator;}
		  else if(cmd == 14) { c_commutation <: iUmotP;  		}
		  else if(cmd == 15) { c_commutation <: iUmotMotor;  	}
		  else if(cmd == 16) { c_commutation <: iPositionAbsolut; }

		  else if(cmd == 17) { c_commutation <: iSetSpeed; 		}
		  else if(cmd == 18) { c_commutation <: iActualSpeed;   }
		  else if(cmd == 19) { c_commutation <: idiffSpeed; 	}
		  else if(cmd == 20) { c_commutation <: iStep1; 		}
		  else if(cmd == 21) { c_commutation <: iIdPeriod2;  	}
		  else if(cmd == 22) { c_commutation <: iIqPeriod2; 	}
		  break;
		default:
		  break;
		}// end select
	}// end while(1)
}// end function






void commutation(chanend c_adc, chanend  c_commutation,  chanend c_hall, chanend c_pwm_ctrl)
{  //init sine-commutation and set up a4935

	  const unsigned t_delay = 300*USEC_FAST;
	  timer t;
	  unsigned ts;

	  comm_sine_init(c_pwm_ctrl);
	  t when timerafter (ts + t_delay) :> ts;

	  a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH);
	  t when timerafter (ts + t_delay) :> ts;

	  do_adc_calibration_ad7949(c_adc);
	  comm_sine(c_adc, c_commutation, c_hall, c_pwm_ctrl);
}

