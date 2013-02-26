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
":",
"idiffSpeed1:",
"idiffSpeed2:",
":",
":",
//------------
"AngleFromHall:",
"AnglePWM:",
":",
":",
"AngleDiffPer:",
"MotDirection:",
":",
":",
//--------------
"Field_Set:",				//15
"Field_Actual:",
"Field_Diff:",
"Torque_Set:",
"Torque_Actual:",
"Torque_Diff:",
":",
":",
//------
"a1RMS_adc:",
"a2RMS_adc:",
"VectorCurr:",
"VectorInvPark:",
":",
"PinStateEnc.:",
"PinStateHall:",
":",
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

char cxText[512];


int input_cmd(cmd_data *c )
{
unsigned char cInput=0;
int xx;
char cxText2[64];
int iIndex;

	if( c->varx == 2)
	{
		c->varx = 0;
		iFlag   = 0;

        printf("-----------  parameter 0-63 ----- to change: example: p01 enter --------- \n");
			iIndex=0;
            while(iIndex < 16)
            {
            	sprintf(cxText,"%2d %-17s %6d   %2d %-17s %6d",
        		iIndex,cxParameter[iIndex],c->iMotPar[iIndex],
        		iIndex,cxParameter[iIndex+16],c->iMotPar[iIndex+16]);
            	printf("%s\n",cxText);
        		iIndex++;
            }
	}// end view parameter

	if(c->varx == 1)
	{
		c->varx = 0;
		iFlag   = 0;

		xx=0;
		c->iMotInfo[xx+0] = c->iMotValues[0] / 65536;
		c->iMotInfo[xx+1] = c->iMotValues[0] & 0xFFFF;
		c->iMotInfo[xx+2] = c->iMotValues[1];   			//iUmotBoost
		c->iMotInfo[xx+3] = c->iMotValues[2];
		c->iMotInfo[xx+4] = c->iMotValues[3];
		c->iMotInfo[xx+5] = c->iMotValues[4];
		c->iMotInfo[xx+6] = c->iMotValues[5] / 256;  // iStep
		c->iMotInfo[xx+7] = 0;

		xx=8;
		c->iMotInfo[xx+0] = c->iMotValues[6]/65536;
		c->iMotInfo[xx+1] = c->iMotValues[6] & 0xFFFF;
		c->iMotInfo[xx+2] = c->iMotValues[7];
		c->iMotInfo[xx+3] = c->iMotValues[8];
		c->iMotInfo[xx+4] = c->iMotValues[9];
		c->iMotInfo[xx+5] = c->iMotValues[10];
		c->iMotInfo[xx+6] = c->iMotValues[11];
		c->iMotInfo[xx+7] = 0;

		xx=16;
		c->iMotInfo[xx+0] = c->iMotValues[12] /65536;		// AngleFromHall
		c->iMotInfo[xx+1] = c->iMotValues[12] & 0xFFFF;     // AnglePWM
		c->iMotInfo[xx+2] = c->iMotValues[14];
		c->iMotInfo[xx+3] = c->iMotValues[15];
		c->iMotInfo[xx+4] = c->iMotValues[16];		//AngleDiffPer
		c->iMotInfo[xx+5] = c->iMotValues[17];		// MotDirection
		c->iMotInfo[xx+6] = 0;
		c->iMotInfo[xx+7] = 0;

		xx=24;
		c->iMotInfo[xx+0] = c->iMotValues[18];  //Field_Set
		c->iMotInfo[xx+1] = c->iMotValues[19];  //Field_Actual
		c->iMotInfo[xx+2] = c->iMotValues[20];  //Field_Diff
		c->iMotInfo[xx+3] = c->iMotValues[21];
		c->iMotInfo[xx+4] = c->iMotValues[22];
		c->iMotInfo[xx+5] = c->iMotValues[23];
		c->iMotInfo[xx+6] = c->iMotValues[23];
		c->iMotInfo[xx+7] = 0;

		xx=32;
		c->iMotInfo[xx+0] = c->iMotValues[24]; //a1RMS_adc
		c->iMotInfo[xx+1] = c->iMotValues[25]; //a2RMS_adc
		c->iMotInfo[xx+2] = c->iMotValues[26]; //VectorCurr
		c->iMotInfo[xx+3] = c->iMotValues[27]; //VectorInvPark
		c->iMotInfo[xx+4] = 0;
		c->iMotInfo[xx+5] = c->iMotValues[29] / 256;  // PinStateHall
		c->iMotInfo[xx+6] = c->iMotValues[29] & 0xFF; // PinStateEnc
		c->iMotInfo[xx+7] = 0;



		c->iMotInfo[40] = c->iMotValues[30];  //iPositionEncoder
		c->iMotInfo[41] = c->iMotValues[31];  //iHallPositionAbsolut


		 	 	fa1RMS = c->iMotInfo[24]; fa1RMS/=264;
		 	 	fa2RMS = c->iMotInfo[25]; fa2RMS/=264;

		 	 	xx = c->iMotValues[17] & 0xFF;
		 	 	if(xx==0) sprintf(cxText2,"motor stop");
		 	 	if(xx==1) sprintf(cxText2,"motor CCW speed: %d RPM",c->iMotValues[7]);
		 	 	if(xx==-1)sprintf(cxText2,"motor  CW speed: %d RPM",c->iMotValues[7]);

		 	 	xx = c->iMotValues[5] & 0xFF;
		 	 	if(xx <= 1) 	sprintf(cxText,"-----------------------                 SPEED Control      ---- +/- RPM  ----- %s ---------------------------",cxText2);
		 	 	if(xx == 2)		sprintf(cxText,"-----------------------                 TORQUE Control     ---- t+200----------------------------------------");
		 	 	if(xx == 3)		sprintf(cxText,"-----------------------                 POSITION Control   ---- m+500 ----------------------------------------");
		 	 	printf("%s\n",cxText);

		 	 	xx=0;
		    	sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %4.2fA",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],fa1RMS);  // RMS
		    	printf("%s\n",cxText);

		 	 	xx=1;
		    	sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %4.2fA",
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
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d   %8d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],c->iMotInfo[41]);
			    printf("%s\n",cxText);

/*
		 	 	xx=7;   // reserve
			    sprintf(cxText,"%-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d    %-15s%5d  %8d",
				cxInfo[xx+0],c->iMotInfo[xx+0],
				cxInfo[xx+8],c->iMotInfo[xx+8],
				cxInfo[xx+16],c->iMotInfo[xx+16],
				cxInfo[xx+24],c->iMotInfo[xx+24],
				cxInfo[xx+32],c->iMotInfo[xx+32],);
			    printf("%s\n",cxText);
*/
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
		    c->iMotCommand[6]=iTemp;
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
			scanf("%c",&iTemp);
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

