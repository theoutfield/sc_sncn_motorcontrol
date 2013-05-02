/*
 * write.c
 *
 *  Created on: May 2, 2012
 *      Author: pkanajar @ Synapticon
 *              04/2013 orgler
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "set_cmd.h"

//int iIndex =   1;
int iFlag=1;
int iTemp;


int input_cmd(cmd_data *c )
{
unsigned char cInput=0;

	if(iFlag)
	{
	printf("\n Choose to set: \n");
	printf(" +/-RPM Speed RPM 0-4000 Speed: %d \n",c->iMotCommand[0]);
	printf(" ENTER=info Motor: 0(null):STOP\n");
	}
    iFlag=1;

	while(1)
	{
    scanf("%c", &cInput);

	switch(cInput)
	{
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
		default:
			break;
	}
 }// end while 1
 return 0;
}

