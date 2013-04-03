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


char cxInfo[64][20]={\
"UmotProfile:",
"UmotBoost:",
"UmotIntegral:",
"UmotProport:",
"UmotMotor:",
"HoldingTorq:",
"Step:",
":",
//----------
"SetSpeedUser:",
"SetSpeedRamp:",
"SpeedActual:",
"diffSpeed1:",
"diffSpeed2:",
":",
":",
":",
//------------
"AngleHall",
"AngleEncoder",
"AngleRotor",
"AnglePWM",
"DiffCalculated",
":",
"TorqueProp",
":",
//--------------
"TorqueSet",
"iIq:",				//15
"iIqperiod2:",
"iTorqueDiff1",
"iTorqueDiff2",
"TorqueIntegr.",
"TorqueLimit",
":",
//------
"a1RMS_adc:",
"a2RMS_adc:",
"VectorCurr:",
"VectorInvPark:",
"PositionAbs.",
"PinStateEnc.:",
"PinStateHall:",
":",
};



char cxParameter[32][32]={\
"RpmMotorMax",
"RpmUmotMax",
"UmotStart",
"UmotBoost",
"4:",
"AngleUser",
"",
"Encoder resolution",
"Encoder zero",
"9:",
"RmsLimit",
"RmsMaxPwmOff",
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
"ramp acc",
"ramp dec",
"RampSmoothFactor",
"27:",
"PositionSpeedMax",
"PositionSpeedMin",
"30",
"31",
};

char cxText[256];


int ReadMotorValue(cmd_data *cc,int iIndex, int iModus)
{
int iValue;

    iValue = cc->iMotValues[iIndex];
	if(iModus==1)	iValue /= 65536;	// Highword
	else iValue &= 0xFFFF;				// LowWord

	if(iValue  > 32768) iValue |= 0xFFFF0000;
	return(iValue);
}


int input_cmd(cmd_data *c )
{
unsigned char cInput=0;
int xx;
char cxText2[256];
char chx;
int iIndex;

int iStep1;
int iControlFOC;
int iMotDirection;
int iStepRamp;

	if( c->varx == 2)
	{
		c->varx = 0;
		iFlag   = 0;

        printf("-----------  parameter 0-63 ----- to change: example: p01 enter --------- \n");
			iIndex=0;
            while(iIndex < 16)
            {
            	sprintf(cxText,"%2d %-20s %6d      %2d %-20s %6d",
        		iIndex,cxParameter[iIndex],c->iMotPar[iIndex],
        		iIndex+16,cxParameter[iIndex+16],c->iMotPar[iIndex+16]);
            	printf("%s\n",cxText);
        		iIndex++;
            }
	}// end view parameter

	if(c->varx == 1)
	{
		c->varx = 0;
		iFlag   = 0;

		iMotDirection = ReadMotorValue(c,5,1);
		iStepRamp     = iMotDirection/256;
		iMotDirection &= 0xFF;

		iStep1        = ReadMotorValue(c,5,0);
		iControlFOC   = iStep1 & 0xFF;
		iStep1        /= 256;

		xx=0;
		c->iMotInfo[0] = ReadMotorValue(c,0,1);
		c->iMotInfo[1] = c->iMotValues[1];   		//iUmotBoost
		c->iMotInfo[2] = ReadMotorValue(c,0,0);
		c->iMotInfo[3] = c->iMotValues[2];
		c->iMotInfo[4] = c->iMotValues[3];
		c->iMotInfo[5] = c->iMotValues[4];
		c->iMotInfo[6] = iStep1;
		c->iMotInfo[7] = 0;

		xx=8;
		c->iMotInfo[8] = ReadMotorValue(c,6,1);		//SetSpeedUser
		c->iMotInfo[9] = ReadMotorValue(c,6,0);  	//SetSpeedRamp
		c->iMotInfo[10] = c->iMotValues[7];       	//SpeedActual
		c->iMotInfo[11] = ReadMotorValue(c,8,1);	//diffSpeed1
		c->iMotInfo[12] = ReadMotorValue(c,8,0);	//diffSpeed2
		c->iMotInfo[13] = c->iMotValues[10];		//iPositionDec
		c->iMotInfo[14] = c->iMotValues[11];		//iPulsCountAcc
		c->iMotInfo[15] = 0;

		xx=16;
		c->iMotInfo[xx+0] = ReadMotorValue(c,12,1);   //  AngleFromHall
		c->iMotInfo[xx+1] = ReadMotorValue(c,12,0);   //  AnglePWM
		c->iMotInfo[xx+2] = ReadMotorValue(c,13,1);   //c->iMotValues[13];		  //  AngleDiffPer
		c->iMotInfo[xx+3] = ReadMotorValue(c,13,0);   //c->iMotValues[14];		  //  RampAccValue
		c->iMotInfo[xx+4] = c->iMotValues[14];
		c->iMotInfo[xx+5] = c->iMotValues[15];
		c->iMotInfo[xx+6] = c->iMotValues[17];        // TorqueLimit
		c->iMotInfo[xx+7] = 0;

		xx=24;
		c->iMotInfo[xx+0] = c->iMotValues[18];
		c->iMotInfo[xx+1] = c->iMotValues[19];
		c->iMotInfo[xx+2] = c->iMotValues[20];
		c->iMotInfo[xx+3] = c->iMotValues[21];
		c->iMotInfo[xx+4] = c->iMotValues[22];
		c->iMotInfo[xx+5] = c->iMotValues[23];
		c->iMotInfo[xx+6] = c->iMotValues[16];
		c->iMotInfo[xx+7] = 0;

		xx=32;
		c->iMotInfo[32] = c->iMotValues[24];
		c->iMotInfo[33] = c->iMotValues[25];
		c->iMotInfo[34] = c->iMotValues[26];
		c->iMotInfo[35] = c->iMotValues[27];
		c->iMotInfo[36] = c->iMotValues[28];
		c->iMotInfo[37] = c->iMotValues[29] & 0xFF; // PinStateEnc
		c->iMotInfo[38] = c->iMotValues[29] / 256;  // PinStateHall
		c->iMotInfo[39] = 0;

		c->iMotInfo[40] = c->iMotValues[30];  //iPositionEncoder
		c->iMotInfo[41] = c->iMotValues[31];  //iHallPositionAbsolut


		 	 	fa1RMS = c->iMotInfo[32]; fa1RMS/=264;
		 	 	fa2RMS = c->iMotInfo[33]; fa2RMS/=264;


		 	 	if(iMotDirection == 0)  sprintf(cxText2,"motor stop");
		 	 	if(iMotDirection == 1)  sprintf(cxText2,"motor CCW speed: %d RPM",c->iMotValues[7]);
		 	 	if(iMotDirection ==0xFF)sprintf(cxText2,"motor  CW speed: %d RPM",c->iMotValues[7]);

		 	 	if(iControlFOC == 0) 	sprintf(cxText,"--------------------------------------- F0:   Umot Control   ----------------------------------------------------");
		 	 	if(iControlFOC == 1) 	sprintf(cxText,"--------------------------------------- F1:   SPEED Control      ---- +/- RPM  ------ %s ---------------------------",cxText2);
		 	 	if(iControlFOC == 2)	sprintf(cxText,"--------------------------------------- F2: Speed and  TORQUE Control     ---- t+200 +RPM -------------------------------------");
		 	 	if(iControlFOC == 3)	sprintf(cxText,"--------------------------------------- F3: POSITION Control ------ m+500 ----------------------------------------");
		 	 	if(iControlFOC == 4)	sprintf(cxText,"--------------------------------------- F4: Sensorless Control --------------------------------------------------------");
		 	 	if(iControlFOC > 4) 	sprintf(cxText,"--------------------------------------  F%d:  iControlFOC  ------------------------------------------------------",iControlFOC);


		 	 	printf("%s\n",cxText);

		 	 	xx=0;
		    	sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d     %5.2fA",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],fa1RMS);  // RMS
		    	printf("%s\n",cxText);

		 	 	xx=1;
		    	sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d     %5.2fA",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],fa2RMS);  // RMS
		    	printf("%s\n",cxText);

		 	 	xx=2;
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32]);
			    printf("%s\n",cxText);

		 	 	xx=3;
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32]);
			    printf("%s\n",cxText);

		 	 	xx=4;
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32]);
			    printf("%s\n",cxText);

		 	 	xx=5;
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d   %8d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],c->iMotInfo[40]);
			    printf("%s\n",cxText);

		 	 	xx=6;
			    sprintf(cxText,"%-15s%5d    %-10s%10d    %-12s%8d    %-12s%8d    %-15s%5d   %8d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],c->iMotInfo[41]);
			    printf("%s\n",cxText);
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

		case  '0':
			c->iMotCommand[0]=0;
	    	return(1);
			break;


		case 'h':		// holding torque
		case 'H':
			scanf("%d",&iTemp);
		    c->iMotCommand[3]=iTemp;
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
		    c->iMotCommand[2]=iTemp;
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

   //------------------ parameter -----------------------
		case 'v':		// view all parameter
		case 'V':
			scanf("%c",&chx);
			c->varx = 2;
			return(0);
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
			c->varx = 2;
			return(iIndex+96);     // send to motor
			}
			break;

		default:
			break;
	}
 }// end while 1
 return 0;
}

