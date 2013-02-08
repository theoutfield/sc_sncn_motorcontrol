/*
 * write.c
 *
 *  Created on: May 2, 2012
 *      Author: pkanajar @ Synapticon
 *              30.01.2013 orgler
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "set_cmd.h"


int iSetValueSpeedRpm=    0;
int iPwmOnOff        =    1;
int iHoldingTorque   =    0;
int iTorqueSetValue  =    0;
int iControlFOCcmd   =    0;
int iIndex =   1;
float fa1RMS,fa2RMS,fa3RMS;
int iFlag=1;
int iTemp;


char cxInfo[32][20]={\
"UmotProfile:",
"UmotIntegrator:",
"UmotProp:",
"UmotMotor:",
"ActualSpeed:",
"iStep1:",
//----
"FOC:",
"SetUserSpeed:",
"SetSpeed:",
"idiffSpeed1:",
"idiffSpeed2:",
"HoldingTorq:",
//-----
"AngleFromHall:",
"AnglePWM:",
":",
":",
"AngleDiffPer:",
"MotDirection:",
//--------------
"Field_Set:",				//15
"Field_Actual:",
"Field_Diff:",
"Torque_Set:",
"Torque_Actual:",
"Torque_Diff:",
//------
"a1RMS_adc:",
"a2RMS_adc:",
"VectorCurr:",
"VectorInvPark:",
"PinStateEnc.:",
"PinStateHall:",
//--
":",
":"
};



char cxParameter[32][32]={\
"0 iParRpmMotorMax",
"1 iParDefSpeedMax",
"2 iParRPMreference",
"3 iParAngleUser",
"4 iParAngleFromRPM",
"5 iParUmotBoost",
"6 iParUmotStart",
"7 iParSpeedKneeUmot",
"8 iParAngleCorrVal",
"9 iParAngleCorrMax",
"10 iParRmsLimit",
"11",
"12",
"13",
"14",
"15",
"16 iParHysteresisPercent",
"17 iParDiffSpeedMax",
"18 iParUmotIntegralLimit",
"19 iParPropGain",
"20 iParIntegralGain",
"21",
"22",
"23",
"24",
"25 test1",
"26 test2",
"27 test3",
"28 test4",
"29 test5",
"30",
"31",
};


void SetMotorValue(cmd_data *c)
{
    c->iControlFOCcmd          = iControlFOCcmd;
    c->iTorqueSetValue      = iTorqueSetValue;
	c->iHoldingTorque       = iHoldingTorque;
	c->iSetValueSpeedRpm    = iSetValueSpeedRpm;
	c->iPwmOnOff 			= iPwmOnOff;
}



int input_cmd(cmd_data *c )
{
unsigned char cInput=0;
int xx;


	if( c->varx == 2)
	{
		c->varx = 0;
		iFlag   = 0;
				iTemp=0;
	            while(iTemp < 32)
	            {
				printf(" p%-2d: %5d  ",iTemp,c->iMotPar[iTemp]); iTemp++;
				printf(" p%2d: %5d  ",iTemp,c->iMotPar[iTemp]);  iTemp++;
				printf(" p%2d: %5d  ",iTemp,c->iMotPar[iTemp]);  iTemp++;
				printf(" p%2d: %5d\n",iTemp,c->iMotPar[iTemp]);  iTemp++;
	            }
	}

	if(c->varx == 1)
	{
		c->varx = 0;
		iFlag   = 0;
		 fa1RMS = c->iMotValues[24]; fa1RMS/=264;
		 fa2RMS = c->iMotValues[25]; fa2RMS/=264;

		xx=0; while(xx < 24){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		xx=24; printf("%-15s%5d    %4.2fA",cxInfo[xx],c->iMotValues[xx],fa1RMS);  // RMS
		printf("\n");

		xx=1; while(xx < 25){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		xx=25;  printf("%-15s%5d    %4.2fA",cxInfo[xx],c->iMotValues[xx],fa2RMS);  // RMS
		printf("\n");

		xx=2; while(xx <= 26){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		printf("\n");

		xx=3; while(xx <= 27){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		printf("\n");

		xx=4; while(xx < 28){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		xx=28; printf("%-15s%5d  %8d",cxInfo[xx],c->iMotValues[xx],c->iMotValues[30]);
		printf("\n");

		xx=5; while(xx < 29){	 printf("%-15s%5d    ",cxInfo[xx],c->iMotValues[xx]); xx += 6; }
		xx=29; printf("%-15s%5d  %8d",cxInfo[xx],c->iMotValues[xx],c->iMotValues[31]);
		printf("\n");
	}


	if(iFlag)
	{
	printf("\n Choose to set:  pxx=parameter v=view parameter \n");
	printf(" h ->holding torque 0-800\n");
	printf(" +/-RPM Speed RPM 0-4000 Speed: %d \n",iSetValueSpeedRpm);
	printf(" ENTER=info Motor: 0(null):STOP\n");
	}
    iFlag=1;

	while(1)
	{
    scanf("%c", &cInput);

	switch(cInput)
	{
	case 0x0A:
	  	c->varx = 1;   // view infos
	  	iFlag=0;
		return(0);
		break;

		case 'v':		// view all parameter
		case 'V':
			c->varx = 2;
			return(0);
			break;

		case  '0':
			iSetValueSpeedRpm = 0;
			SetMotorValue(c);
	    	return(1);
			break;


		case 'h':
		case 'H':
			scanf("%d",&iHoldingTorque);
			SetMotorValue(c);
			return(1);
			break;


		case '+':
			scanf("%d",&iSetValueSpeedRpm);
			SetMotorValue(c);
			return(1);
			break;

		case '-':
			scanf("%d",&iSetValueSpeedRpm);
			iSetValueSpeedRpm    = -iSetValueSpeedRpm;
			SetMotorValue(c);
			return(1);
			break;

		case 'x':			// PWM OFF
		case 'X':
		    iPwmOnOff = 0;
		    iSetValueSpeedRpm = 0;
			SetMotorValue(c);
			return(1);
			break;

		case 'z':		// PWM ON
		case 'Z':
		    iPwmOnOff = 1;
		    iSetValueSpeedRpm = 0;
			SetMotorValue(c);
			return(1);
			break;

		case 't':			// set torque value
		case 'T':
		    scanf("%d",&iTemp);
		    iTorqueSetValue 		= iTemp;
			SetMotorValue(c);
			return(1);
			break;

		case 'f':			// set torque value
		case 'F':
		    scanf("%d",&iTemp);
		    iControlFOCcmd 			= iTemp;
			SetMotorValue(c);
			return(1);
			break;

		case 'p':
		case 'P':
			iIndex = -1;
			scanf("%d",&iIndex);
			if(iIndex >= 0 && iIndex < 32)
			{
			printf(" p%d:  %s:%d \n",iIndex,cxParameter[iIndex],c->iMotPar[iIndex]);
			scanf("%d",&iTemp);
			c->iMotPar[iIndex] = iTemp;
			iFlag=0;
			c->var1 = c->iMotPar[iIndex];
			return(iIndex+96);
			}
			break;
		default:

			break;
	}
 }// end while 1
 return 0;
}

