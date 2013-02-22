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
"RpmMotorMax",
"DefSpeedMax",
"RPMreference",
"AngleUser",
"AngleFromRPM",
"UmotBoost",
"UmotStart",
"SpeedKneeUmot",
"AngleCorrVal",
"AngleCorrMax",
"RmsLimit",
"11",
"12",
"13",
"14",
"15",
"HysteresisPercent",
"DiffSpeedMax",
"UmotIntegralLimit",
"PropGain",
"IntegralGain",
"21",
"22",
"23",
"24",
"test1",
"test2",
"test3",
"test4",
"test5",
"30",
"31",
};




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
	printf(" +/-RPM Speed RPM 0-4000 Speed: %d \n",c->iMotCommand[0]);
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
			c->iMotCommand[0]=0;
	    	return(1);
			break;


		case 'h':		// holding torque
		case 'H':
			scanf("%d",&iTemp);
		    c->iMotCommand[7]=iTemp;
			return(1);
			break;

		case 'm':
		case 'M':
			scanf("%d",&iTemp);
		    c->iMotCommand[10]=iTemp;
			return(1);
			break;

		case 't':			// set torque value
		case 'T':
		    scanf("%d",&iTemp);
		    c->iMotCommand[10]=iTemp;
			return(1);
			break;

		case 'f':			// set iControlFOC
		case 'F':
		    scanf("%d",&iTemp);
		    c->iMotCommand[1]=iTemp;
			return(1);
			break;

		case '+':
		    scanf("%d",&iTemp);
		    c->iMotCommand[0]=iTemp;
			return(1);
			break;

		case '-':
		    scanf("%d",&iTemp);
		    if(iTemp > 0)iTemp = -iTemp;
		    c->iMotCommand[0]=iTemp;
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

