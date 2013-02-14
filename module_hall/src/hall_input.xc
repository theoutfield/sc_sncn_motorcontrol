/**
 * Module:  module_hall
 * Version: 1v0alpha2

 * orgler@tin.it synapticon 01/2013
  *
 **/                                   
#include "hall_input.h"
#include <stdlib.h>
#include <print.h>
#include <stdint.h>
#include "refclk.h"
#include "dc_motor_config.h"
#include <xscope.h>

extern out port p_ifm_ext_d3;

void run_hall( chanend c_hall, port in p_hall, port in p_encoder)
 {
  timer tx;
  unsigned ts;					// newest timestamp
  unsigned cmd;
  int iTemp;

  int angle1;		        // newest angle (base angle on hall state transition)
  int angle2;
  int delta_angle;
  int iAngleLast;
  int iAngleDiff;

  unsigned iCountMicroSeconds;
  int iHallActualSpeed=0;
  unsigned iPeriodMicroSeconds;

  int iCountTransitionSum;
  int iCountTransitionNew;
  int iCountTransitionFiltered;
  int iCountTransitionEstimated;
  int iTimeCountOneTransition=0;
  int iTimeSaveOneTransition=0;


  unsigned iHallStateNew;			// newest hall state
  unsigned iHallStateNew_last;
  unsigned new1,new2;
  unsigned uHallNext,uHallPrevious;
  int xreadings  =0;
  int iPosAbsolut=0;
  int iHallError=0;
  int iHalldirection    = 0;
  int iNrHallPulses     = 1;
  int iCountHallPulses  = 0;
  int iAngleDeltaSum 	= 0;
  int iAngleDeltaValue  = 682;

//=========== encoder =======================
  int iStepEncoder		=0;
  int iEncState1		=0;
  int iEncState2		=0;
  int iEncoderPinState;
  int iEncoderReferenz;
  int iEncoderStateNew;
  int iEncoderStateOld;
  int iEncoderError		=0;
  int iPosEncoder		=0;
  int iEncoderDirection =0;
  int iEncoderNext=0;
  int iEncoderPrevious=0;
unsigned char cFlagX=0;

  tx :> ts;  // first value

  p_hall :> new1;
  iHallStateNew      = new1;
  iHallStateNew_last = new1;

  //********************* LOOP 1µsec ****************************
  while(1) {

	  cFlagX ^= 0x01;
	  p_ifm_ext_d3 <: cFlagX;
	  switch(xreadings)
	  {
		  case 0: p_hall :> new1; new1 &= 0x07; xreadings++;
		  	  	  break;
		  case 1: p_hall :> new2; new2 &= 0x07;
				  if(new2 == new1) xreadings++;
				  else xreadings=0;
				  break;
		  case 2: p_hall :> new2; new2 &= 0x07;
				  if(new2 == new1) iHallStateNew = new2;
				  else xreadings=0;
				  break;
	  }

	  //==================================== encoder ===================================

	  switch(iStepEncoder)
	  {
		  case 0: p_encoder :> iEncState1; iEncState1 &= 0x07; iStepEncoder++;
		  	  	  break;
		  case 1: p_encoder :> iEncState2; iEncState2 &= 0x07;
				  if(iEncState2 == iEncState1) iStepEncoder++;
				  else iStepEncoder=0;
				  break;
		  case 2: p_encoder :> iEncState2; iEncState2 &= 0x07;
				  if(iEncState2 == iEncState1) iEncoderPinState = iEncState2;
				  else iStepEncoder=0;
				  break;
	  }

	  iEncoderReferenz = iEncoderPinState & 0x04;
	  iEncoderStateNew = iEncoderPinState & 0x03;

#define defState0  0
#define defState1  2
#define defState2  3
#define defState3  1



	 if(iEncoderStateOld != iEncoderStateNew)
	 {

		 if(iEncoderStateNew == iEncoderNext)    {iPosEncoder++; iEncoderDirection =  1;  }
	     if(iEncoderStateNew == iEncoderPrevious){iPosEncoder--; iEncoderDirection = -1;  }

	      switch(iEncoderStateNew)
	  	  {
			  case defState0:   iEncoderNext=defState1; iEncoderPrevious=defState3;  break;
			  case defState1:   iEncoderNext=defState2; iEncoderPrevious=defState0;  break;
			  case defState2:   iEncoderNext=defState3; iEncoderPrevious=defState1;  break;
			  case defState3:   iEncoderNext=defState0; iEncoderPrevious=defState2;  break;

			  default: iEncoderError++; break;
	  	  }
	      iEncoderStateOld = iEncoderStateNew;
	 }



	  iCountMicroSeconds++;   		// period in µsec
	  iTimeCountOneTransition++;
	  iAngleDeltaSum += iAngleDeltaValue;

      if(iHallStateNew != iHallStateNew_last)
      {
 	      if(iHallStateNew == uHallNext)    {iPosAbsolut++; iHalldirection = 1;  }
	      if(iHallStateNew == uHallPrevious){iPosAbsolut--; iHalldirection =-1;  }

	      //if(iHalldirection >= 0) // CW  3 2 6 4 5 1

#define defHallState0 3
#define defHallState1 2
#define defHallState2 6
#define defHallState3 4
#define defHallState4 5
#define defHallState5 1


	      switch(iHallStateNew)
	  	  {
			  case defHallState0: angle1 =     0;  uHallNext=defHallState1; uHallPrevious=defHallState5;  break;
			  case defHallState1: angle1 =   682;  uHallNext=defHallState2; uHallPrevious=defHallState0;  break;   //  60
			  case defHallState2: angle1 =  1365;  uHallNext=defHallState3; uHallPrevious=defHallState1;  break;
			  case defHallState3: angle1 =  2048;  uHallNext=defHallState4; uHallPrevious=defHallState2;  break;   // 180
			  case defHallState4: angle1 =  2730;  uHallNext=defHallState5; uHallPrevious=defHallState3;  break;
			  case defHallState5: angle1 =  3413;  uHallNext=defHallState0; uHallPrevious=defHallState4;  break;   // 300 degree
			  default: iHallError++; break;
	  	  }


	      iCountHallPulses++;
	      if(iCountHallPulses >= iNrHallPulses)
	      {
	    	  iCountHallPulses 		  = 0;
	    	  iPeriodMicroSeconds     = iCountMicroSeconds;
	    	  iCountMicroSeconds      = 0;
	    	  iHallActualSpeed        = 0;
	    	  if(iPeriodMicroSeconds)
	    	  {
	    	  iHallActualSpeed  = 10000000;
	    	  iHallActualSpeed *= iNrHallPulses;
	    	  iHallActualSpeed /= iPeriodMicroSeconds;    	// period in µsec
	    	  iHallActualSpeed /= POLE_PAIRS; 				// pole pairs
	    	  }
	    	  if(iHalldirection == -1) iHallActualSpeed = -iHallActualSpeed;

	    	  iHallActualSpeed &= 0x00FFFFFF;
	    	  iHallActualSpeed |= 0xAA000000;
	      }
	      switch(iNrHallPulses)
	      {
	      case 1:  if(iPeriodMicroSeconds < 7143)    iNrHallPulses = 2;
	    	  	   break;
	      case 2:  if(iPeriodMicroSeconds > 9000)    iNrHallPulses = 1;
	      	  	   if(iPeriodMicroSeconds < 1429)    iNrHallPulses = 6;
	      	  	   break;
	      case 6:  if(iPeriodMicroSeconds > 1786)    iNrHallPulses = 2;
	      	       break;
	      default: iNrHallPulses=1; break;
	      }

	   iTimeSaveOneTransition  = iTimeCountOneTransition;
	   iCountTransitionNew     = iTimeCountOneTransition;
	   iTimeCountOneTransition = 0;
       delta_angle             = 0;
       iHallStateNew_last  	   = iHallStateNew;

        if(iHalldirection==1){
    	if(angle1 < 1024 && iAngleLast > 3072)
    		iAngleDiff  = (angle1 + 4096) - iAngleLast;
    	else
    		iAngleDiff  = angle1 - iAngleLast;
       }
      }// end (iHallStateNew != iHallStateNew_last
     //==================================================================


	#define defPeriodMax 1000000  //1000msec
		if(iCountMicroSeconds > defPeriodMax)
			{
			iCountMicroSeconds = defPeriodMax;
			iHallActualSpeed   = 0xAA000000;
			}


 		if(iTimeCountOneTransition)
 		{

 		if(iTimeCountOneTransition == 1)
 		{
 			iCountTransitionSum      -= iCountTransitionFiltered;
 			iCountTransitionSum      += iCountTransitionNew;
 			iCountTransitionFiltered  = iCountTransitionSum/4;
 			iCountTransitionEstimated = (iCountTransitionFiltered*3)/4 + iCountTransitionNew/4;

 			iTemp = iCountTransitionNew - iCountTransitionFiltered;
 			iTemp *= 682;
 			iTemp /= iCountTransitionFiltered;
 			iAngleDeltaSum += iTemp;
 			if(iAngleDeltaSum > 692) iAngleDeltaSum = 692;
			if(iAngleDeltaSum < 672) iAngleDeltaSum = 672;
 		/*	 xscope_probe_data(8,iCountTransitionNew);
 		 	 xscope_probe_data(9,iAngleDiff);
 		 	 xscope_probe_data(10,iAngleDeltaSum);*/
 		}
// 		if(iCountTransitionEstimated)
//		delta_angle = (682 *iTimeCountOneTransition)/iCountTransitionEstimated; //iTimeSaveOneTransition;
		if(iCountTransitionEstimated)
		delta_angle = iAngleDeltaSum/iCountTransitionEstimated; //iTimeSaveOneTransition;
 		}



	  if(delta_angle >= 680) delta_angle = 680;

	  if(iTimeCountOneTransition > 50000) iHalldirection = 0;

	  angle2 = angle1;
      if(iHalldirection == 1)  angle2 += delta_angle;
      if(iHalldirection == -1) angle2 -= delta_angle;
      angle2 &= 0x0FFF;    // 4095
      iAngleLast  = angle2;



//	  tx :> ts;
 	  tx when timerafter(ts + 250) :> ts;

	#pragma ordered

 	    select {
			case c_hall :> cmd:
				  if  (cmd == 1) { c_hall <: angle2; }
			 else if  (cmd == 2) { c_hall <: iHallActualSpeed;   iHallActualSpeed &= 0x00FFFFFF; }
			 else if  (cmd == 3) { c_hall <: iPosAbsolut;  		}
			 else if  (cmd == 4) { c_hall <: iHallError;   		}
			 else if  (cmd == 5) { c_hall <: iHallStateNew;   	}
			 else if  (cmd == 6) { c_hall <: iEncoderPinState;  }
			 else if  (cmd == 7) { c_hall <: iPosEncoder;   	}
			break;
			default:
			  break;
 	    }// end of select

  }// end while 1
}





