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


int iSetValueSpeedRpm=   0;
int iPwmOnOff        =   1;
int iIndex =   1;
float fa1RMS,fa2RMS,fa3RMS;
int iFlag=1;
int iTemp;

char cxInfo[32][20]={\
"iUmotSquare:",
"iUmotLinear:",
"iUmotIntegrator:",
"iUmotProp:",
"iUmotMotor:",
"iSetSpeed:",
"iActualSpeed:",
"idiffSpeed:",
"idiffSpeed2:",
":",
"iStep:",
"PositionAbs:",
":",
":",
":",
"field__per:",
"torque_per:",
"AngleDiffPer:",
"AngleCorrect:",
":",
"ia1RMS:",
"ia2RMS:",
"ia3RMS:",
":",
":",
":",
":",
":",
":",
":",
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
"25",
"26",
"27",
"28",
"29",
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
				printf(" p%2d: %5d  ",iTemp,c->iMotPar[iTemp]); iTemp++;
				printf(" p%2d: %5d  ",iTemp,c->iMotPar[iTemp]); iTemp++;
				printf(" p%2d: %5d\n",iTemp,c->iMotPar[iTemp]); iTemp++;
	            }
	}

	if( c->varx == 1)
	{
		c->varx = 0;
		iFlag   = 0;
		 fa1RMS = c->iMotValues[20]; fa1RMS/=264;
		 fa2RMS = c->iMotValues[21]; fa2RMS/=264;
		 fa3RMS = c->iMotValues[22]; fa3RMS/=264;

		xx=0;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=5;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=10; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=15; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=20; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);  // RMS
		printf("\n");

		xx=1;   printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=6;   printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=11;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=16;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=21;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]); // RMS
		printf("\n");

		xx=2;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=7;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=12; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=17; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=22; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);  // RMS
		printf("\n");

		xx=3;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=8;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=13; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=18; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		printf("a1RMS:            %4.2fA",fa1RMS);
		printf("\n");

		xx=4;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=9;  printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=14; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		xx=19; printf("%-16s%6d  ",cxInfo[xx],c->iMotValues[xx]);
		printf("a2RMS: a3RMS:     %4.2fA  %4.2fA",fa2RMS,fa3RMS);
		printf("\n");
	}


	if(iFlag)
	{
	printf("\nChoose to set:  pxx=parameter v=view parameter \n");
	printf(" +/-RPM Speed RPM 0-4000 Speed: %d \n",iSetValueSpeedRpm);
	printf(" I info Motor: 0(null):STOP\n");
	}
    iFlag=1;

	while(1)
	{
    scanf("%c", &cInput);

	switch(cInput)
	{
		case 'i':
		case 'I':
	    	c->varx = 1;
			return(0);
			break;
		case 'v':			// view all parameter
		case 'V':
			c->varx = 2;
			return(0);
			break;

		case  '0':
			iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case '+':
			scanf("%d",&iSetValueSpeedRpm);
			c->iSetValueSpeedRpm        = iSetValueSpeedRpm;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case '-':
			scanf("%d",&iSetValueSpeedRpm);
			iSetValueSpeedRpm    = -iSetValueSpeedRpm;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case 'x':
		case 'X':
		    iPwmOnOff = 0;
		    iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case 'z':
		case 'Z':
		    iPwmOnOff = 1;
		    iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iPwmOnOff = iPwmOnOff;
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
			return(iIndex+32);
			}
			break;

		default:
			printf("\n");
			break;
	}
 }// end while 1
 return 0;
}

