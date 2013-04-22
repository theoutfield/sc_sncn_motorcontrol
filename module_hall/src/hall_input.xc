#include "hall_input.h"
#include <stdlib.h>
#include <print.h>
#include <stdint.h>
#include "refclk.h"
#include "dc_motor_config.h"
#include <xscope.h>


// iEncoderSpeed = (60 * 1.000.000) / periodeï¿½sec
// 60.000.000 / 400 = 150.000   / 4= 37500
//=========RPM_PWM   1.000.000 * 60 / Periode
//  RPM_Motor = RPM_PWM/POLE_PAIRS
//  60.000.000 /Periode 8314/2*6

#define defHallState0 3
#define defHallState1 2
#define defHallState2 6
#define defHallState3 4
#define defHallState4 5
#define defHallState5 1
#define defPeriodMax 200000  //200msec
#define defHallPeriodMax	  200000  		// 200msec

void run_hall(chanend c_hall, port in p_hall)
{
	timer tx;
	unsigned ts; // newest timestamp
	unsigned cmd;
	int iTemp;


	//============================================
	int iHallDirection = 0;
	int iCountHallPulses = 0;
	int iNrHallPulses = 2;
	int iHallCountMicroSeconds = 0;
	int iHallPeriodMicroSeconds = 0;
	int iHallPeriodNext = defHallPeriodMax;
	int iHallSpeed = 0;
	int iHallSpeedTemp;
	int iHallPeriodeRef = 0;
	int iHallCountPeriodeRef = 0;

	int iHallPulsesLast = 0;
	int iHallDividend;
	int iHallDividend2;

	int iHallCountTransitionSum;
	int iHallCountTransitionNew;
	int iHallCountTransitionFiltered;
	int iHallCountTransitionEstimated;
	int iHallCountOneTransition = 0;

	unsigned iHallStateOld;
	unsigned iHallState1, iHallState2;
	unsigned uHallNext, uHallPrevious;

	int iStepHall = 0;
	int iHallError = 0;
	int iHallAngleDeltaSum = 0;
	int iHallAngleDeltaValue = 682;
	int iHallAngle1; // newest angle (base angle on hall state transition)

	int delta_angle;
	unsigned iHallStateNew; // newest hall state
	int iHallAngle2 = 0;
	int iHallTimeSaveOneTransition = 0;
	int iHallPosAbsolut = 0;
	int iHallStatusMachine = 0;

	tx	:> ts; // first value


	p_hall :> iHallState1;
	iHallStateNew = iHallState1;
	iHallStateOld = -1;
	iHallDividend = 60000000/2;
	iHallDividend /= POLE_PAIRS;


	//******************************************** LOOP 1 Usec **************************************************
	while(1) {

		iHallCountMicroSeconds++; // period in µsec
		iHallCountOneTransition++;
		iHallCountPeriodeRef++;
		iHallAngleDeltaSum += iHallAngleDeltaValue;

		switch(iStepHall)
		{
			case 0: p_hall :> iHallState1; iHallState1 &= 0x07; iStepHall++;
				break;
			case 1: p_hall :> iHallState2; iHallState2 &= 0x07;
				if(iHallState2 == iHallState1)
					iStepHall++;
				else iStepHall=0;
				break;
			case 2: p_hall :> iHallState2; iHallState2 &= 0x07;
				if(iHallState2 == iHallState1)
					iHallStateNew = iHallState2;
				else iStepHall=0;
				break;
			default: break;
		}

		switch(iHallStatusMachine)
		{
			case 0: break;

			case 1: iHallDividend2 = iHallDividend;
				switch(iHallPulsesLast)
				{
					case 1: iHallDividend2 /= 6; break;
					case 2: iHallDividend2 /= 3; break;
					case 3: iHallDividend2 /= 2; break;
					case 4: iHallDividend2 *= 2; iHallDividend2 /= 3; break;
					case 6: break;
					default: break;
				}
				iHallStatusMachine++;
				break;
			case 2: iHallSpeedTemp = 0;
				if(iHallPeriodMicroSeconds)
				{
					iHallSpeedTemp = iHallDividend2 / iHallPeriodMicroSeconds; // period in µsec
				}
				iHallStatusMachine++;
				break;
			case 3: if(iHallDirection < 0) iHallSpeedTemp = 0-iHallSpeedTemp;
				iHallSpeed = iHallSpeedTemp;
				iHallStatusMachine++;
				break;
			case 4: iHallPeriodNext = iHallPeriodMicroSeconds * 256; // add 10%
				iHallStatusMachine++;
				break;
			case 5: iHallPeriodNext /= 230;
				iHallStatusMachine++;
				break;
			case 6: break;
		}

		if(iHallStateNew != iHallStateOld)
		{
			iHallStateOld = iHallStateNew;

			if(iHallStateNew == uHallNext) {iHallPosAbsolut++; iHallDirection++; if(iHallDirection > 3) iHallDirection = +3;}
			if(iHallStateNew == uHallPrevious) {iHallPosAbsolut--; iHallDirection--; if(iHallDirection < -3) iHallDirection = -3;}

			//if(iHallDirection >= 0) // CW  3 2 6 4 5 1

			switch(iHallStateNew)
			{
				case defHallState0: iHallAngle1 = 0; uHallNext=defHallState1; uHallPrevious=defHallState5;
					iHallPeriodeRef = iHallCountPeriodeRef;
					iHallCountPeriodeRef = 0;
					break;
				case defHallState1: iHallAngle1 = 682; uHallNext=defHallState2; uHallPrevious=defHallState0; break; //  60
				case defHallState2: iHallAngle1 = 1365; uHallNext=defHallState3; uHallPrevious=defHallState1; break;
				case defHallState3: iHallAngle1 = 2048; uHallNext=defHallState4; uHallPrevious=defHallState2; break; // 180
				case defHallState4: iHallAngle1 = 2730; uHallNext=defHallState5; uHallPrevious=defHallState3; break;
				case defHallState5: iHallAngle1 = 3413; uHallNext=defHallState0; uHallPrevious=defHallState4; break; // 300 degree
				default: iHallError++; break;
			}// end switch


			iCountHallPulses++;
			if(iCountHallPulses >= iNrHallPulses)
			{
				iHallPeriodMicroSeconds = iHallCountMicroSeconds;
				iHallCountMicroSeconds = 0;
				iHallPulsesLast = iNrHallPulses;
				iHallStatusMachine = 1;
				iCountHallPulses = 0;

				switch(iNrHallPulses)
				{
					case 2: if(iHallPeriodeRef < 9000) iNrHallPulses = 6;
							break;
					case 6: if(iHallPeriodeRef > 12000) iNrHallPulses = 2;
							break;
					default: iNrHallPulses=2;
							break;
				}
			}

			iHallTimeSaveOneTransition = iHallCountOneTransition;
			iHallCountTransitionNew = iHallCountOneTransition;
			iHallCountOneTransition = 0;
			delta_angle = 0;
		}//====================== end (iHallStateNew != iHallStateOld===========================


		if(iHallCountMicroSeconds > defPeriodMax)
		{
			iHallCountMicroSeconds = defPeriodMax;
			iHallSpeed = 0;
			iNrHallPulses = 2;
		}
		else
		if(iHallCountMicroSeconds > iHallPeriodNext)
		{
			iHallPeriodMicroSeconds = iHallCountMicroSeconds;
			iHallPulsesLast = iNrHallPulses;
			iHallStatusMachine = 1;
		}

		if(iHallCountOneTransition)
		{
			if(iHallCountOneTransition == 1)
			{
				iHallCountTransitionSum -= iHallCountTransitionFiltered;
				iHallCountTransitionSum += iHallCountTransitionNew;
				iHallCountTransitionFiltered = iHallCountTransitionSum/4;
				iHallCountTransitionEstimated = (iHallCountTransitionFiltered*3)/4 + iHallCountTransitionNew/4;

				iTemp = iHallCountTransitionNew - iHallCountTransitionFiltered;
				iTemp *= 682;
				if(iHallCountTransitionFiltered) iTemp /= iHallCountTransitionFiltered;
				iHallAngleDeltaSum += iTemp;
				if(iHallAngleDeltaSum > 692) iHallAngleDeltaSum = 692;
				if(iHallAngleDeltaSum < 672) iHallAngleDeltaSum = 672;
			}

			if(iHallCountTransitionEstimated)
			delta_angle = iHallAngleDeltaSum/iHallCountTransitionEstimated;
		}

		if(delta_angle >= 680) delta_angle = 680;

		if(iHallCountOneTransition > 50000) iHallDirection = 0;

		iHallAngle2 = iHallAngle1;
		if(iHallDirection > 0) iHallAngle2 += delta_angle;
		if(iHallDirection < 0) iHallAngle2 -= delta_angle;
		iHallAngle2 &= 0x0FFF;

		//======================end HALL ===================================================================


		tx when timerafter(ts + 500) :> ts; // 250 => 1Usec


		#pragma ordered   // readout of 4 values about 700nsec
		select {
			case c_hall :> cmd:
				if (cmd == 1)
				master
				{	c_hall <: iHallSpeed;
					c_hall <: iHallAngle2;
					c_hall <: iHallPosAbsolut;
					c_hall <: iHallStateNew;
				}
				break;

			default: break;
		}// end of select

	}// end while 1
}

