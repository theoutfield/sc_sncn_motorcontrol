/*
 * write.c
 *
 *  Created on: May 2, 2012
 *      Author: pkanajar @ Synapticon
 *              21.12.2012 orgler
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "set_cmd.h"


int iAngleVariable  = 300;
int iSetValueSpeedRpm=0;
int iPwmOnOff=1;
float fa1RMS,fa2RMS,fa3RMS;

int input_cmd(cmd_data *c )
{
	unsigned char cInput=0;

	if( c->varx == 1)
	{
		c->varx = 0;
		 printf("iUmotSquare:%d iUmotLinear:%d iUmotIntegrator:%d iUmotP:%d iUmotMotor:%d iPositionAbsolut:%d\n",c->var1,c->var2,c->var3,c->var4,c->var5,c->var6);
		 printf("iSetSpeed:%d iActualSpeed:%d idiffSpeed:%d iStep:%d iIdfiltered:%d iIqfiltered:%d\n",c->var7,c->var8,c->var9,c->var10,c->var11,c->var12);
		 printf("ia1RMS:%d ia2RMS:%d ia3RMS:%d\n",c->var13,c->var14,c->var15);
		 fa1RMS = c->var13; fa1RMS/=264;
		 fa2RMS = c->var14; fa2RMS/=264;
		 fa3RMS = c->var15; fa3RMS/=264;
		 printf("a1RMS:%4.2f a2RMS:%4.2f a3RMS:%4.2f\n",fa1RMS,fa2RMS,fa3RMS);

	}
	else
	{
	printf("\nChoose to set \n");
	printf(" Axxx Set angle only     actual:%d \n",iAngleVariable);
	printf(" +/-RPM Speed RPM 0-4000 Speed: %d \n",iSetValueSpeedRpm);
	printf(" I info Motor: M = max H = half 0(null):STOP\n");
	}


	while(1)
	{
    scanf("%c", &cInput);

	switch(cInput)
	{
	case 'a':
		case 'A':
//			printf("Set Angle 0-4095 (0-359 degree) default:0 actual:%d\n",iAngleVariable);
			scanf("%d",&iAngleVariable);
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			return(1);
			break;

		case 'i':
		case 'I':
		//	printf("INFOS: iAngleVariable:%d iSetValueSpeedRpm=%d\n",iAngleVariable,iSetValueSpeedRpm);
			c->varx = 1;
			return(0);
			break;


		case  '0':
		case   32:    // Space
			iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case '+':
		case 's':
		case 'S':				// motor speed +/- 4000 max
	//		printf("SpeedRpm +/- 0-4000 actual:%d\n",iSetValueSpeedRpm);
			scanf("%d",&iSetValueSpeedRpm);
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    		 = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case 'M':
		case 'm':
			if(iSetValueSpeedRpm >= 0)iSetValueSpeedRpm =  4000;
						         else iSetValueSpeedRpm = -4000;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case 'H':
		case 'h':
			if(iSetValueSpeedRpm >= 0)iSetValueSpeedRpm =  2000;
								 else iSetValueSpeedRpm = -2000;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case '-':
//			printf("SpeedRpm +/- 0-4000 actual:%d\n",iSetValueSpeedRpm);
			scanf("%d",&iSetValueSpeedRpm);
			iSetValueSpeedRpm    = -iSetValueSpeedRpm;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    		 = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);			return(1);
			break;

		case 'x':
		case 'X':
		    iPwmOnOff = 0;
		    iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		case 'p':
		case 'P':
		    iPwmOnOff = 1;
		    iSetValueSpeedRpm = 0;
			c->iSetValueSpeedRpm = iSetValueSpeedRpm;
			c->iAngleVariable    = iAngleVariable;
			c->iPwmOnOff = iPwmOnOff;
			return(1);
			break;

		default:
			printf("\n");
			break;
	}
 }// end while 1
 return 0;
}

