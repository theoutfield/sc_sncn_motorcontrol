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

void run_hall( chanend c_hall, port in p_hall, port in p_encoder)
 {
  timer tx;
  unsigned ts;					// newest timestamp
  unsigned cmd;

  unsigned angle1;		        // newest angle (base angle on hall state transition)
  unsigned delta_angle;
  unsigned angle2;

  unsigned iCountMicroSeconds;
  int iHallActualSpeed=0;
  unsigned iPeriodMicroSeconds;
  unsigned iTimeCountOneTransition=0;
  unsigned iTimeSaveOneTransition=0;
  unsigned iHallStateNew;			// newest hall state
  unsigned iHallStateNew_last;
  unsigned new1,new2;
  unsigned uHallNext,uHallPrevious;
  int xreadings  =0;
  int iPosAbsolut=0;
  int iHallError=0;
  int dir=0;
  int iNrHallPulses    = 1;
  int iCountHallPulses = 0;
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


  tx :> ts;  // first value

  p_hall :> new1;
  iHallStateNew      = new1;
  iHallStateNew_last = new1;

  //********************* LOOP 1탎ec ****************************
  while(1) {

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



	  iCountMicroSeconds++;   // period in 탎ec
	  iTimeCountOneTransition++;

      if(iHallStateNew != iHallStateNew_last)
      {
 	      if(iHallStateNew == uHallNext)    {iPosAbsolut++; dir = 1;  }
	      if(iHallStateNew == uHallPrevious){iPosAbsolut--; dir =-1;  }

	      //if(dir >= 0) // CW  3 2 6 4 5 1

	      switch(iHallStateNew)
	  	  {
			  case 3: angle1 =     0;  uHallNext=2; uHallPrevious=1;  break;
			  case 2: angle1 =   682;  uHallNext=6; uHallPrevious=3;  break;   //  60
			  case 6: angle1 =  1365;  uHallNext=4; uHallPrevious=2;  break;
			  case 4: angle1 =  2048;  uHallNext=5; uHallPrevious=6;  break;   // 180
			  case 5: angle1 =  2730;  uHallNext=1; uHallPrevious=4;  break;
			  case 1: angle1 =  3413;  uHallNext=3; uHallPrevious=5;  break;   // 300 degree
			  default: iHallError++; break;
	  	  }


	      iCountHallPulses++;
	      if(iCountHallPulses >= iNrHallPulses)
	      {
	    	  iCountHallPulses = 0;
	    	  iPeriodMicroSeconds     = iCountMicroSeconds;
	    	  iCountMicroSeconds      = 0;
	    	  iHallActualSpeed        = 0;
	    	  if(iPeriodMicroSeconds)
	    	  {
	    	  iHallActualSpeed  = 10000000;
	    	  iHallActualSpeed *= iNrHallPulses;
	    	  iHallActualSpeed /= iPeriodMicroSeconds;    // period in 탎ec
	    	  iHallActualSpeed /= POLE_PAIRS; //Pole pairs
	    	  }
	    	  if(dir == -1) iHallActualSpeed -= iHallActualSpeed;

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

/*
	    if(dir == 1)
	    	if(iHallStateNew_last==1 && iHallStateNew==3) // transition to NULL
	    	{
				 iPeriodMicroSeconds     = iCountMicroSeconds;
				 iCountMicroSeconds      = 0;
				 iHallActualSpeed        = 0;
	    		if(iPeriodMicroSeconds)
	    		{
					iHallActualSpeed = 60000000/iPeriodMicroSeconds;    // period in 탎ec
					iHallActualSpeed /= POLE_PAIRS; //Pole pairs
	    		}
	    		iHallActualSpeed &= 0x00FFFFFF;
	    		iHallActualSpeed |= 0xAA000000;
	    	}



	    if(dir == -1)
	    	if(iHallStateNew_last==3 && iHallStateNew==1)
	    	{
				iPeriodMicroSeconds     = iCountMicroSeconds;
				iCountMicroSeconds      = 0;
				iHallActualSpeed        = 0;
	    		if(iPeriodMicroSeconds)
	    		{
					iHallActualSpeed = 60000000/iPeriodMicroSeconds;
					iHallActualSpeed /= POLE_PAIRS;
					iHallActualSpeed = -iHallActualSpeed;
	    		}
	    		iHallActualSpeed &= 0x00FFFFFF;
	    		iHallActualSpeed |= 0xAA000000;
	    	}
*/
	   iTimeSaveOneTransition  = iTimeCountOneTransition;
	   iTimeCountOneTransition = 0;
       delta_angle             = 0;
       iHallStateNew_last  = iHallStateNew;

      }// end (iHallStateNew != iHallStateNew_last
     //===============================================================


	#define defPeriodMax 1000000  //1000msec
		if(iCountMicroSeconds > defPeriodMax)
			{iCountMicroSeconds = defPeriodMax;
			 iHallActualSpeed   = 0xAA000000;
			 }


 		if(iTimeCountOneTransition)
 			if(iTimeSaveOneTransition)
		delta_angle = (682 *iTimeCountOneTransition)/iTimeSaveOneTransition;


	  if(delta_angle >= 680) delta_angle = 680;

	  if(iTimeCountOneTransition > 50000) dir = 0;

	  angle2 = angle1;
      if(dir == 1)  angle2 += delta_angle;

      if(dir == -1) angle2 -= delta_angle;

      angle2 &= 0x0FFF;    // 4095

//	  tx :> ts;
 	  tx when timerafter(ts + 250) :> ts;

	#pragma ordered
 	    select {
			case c_hall :> cmd:
				  if  (cmd == 1) { c_hall <: angle2; }
			 else if  (cmd == 2) { c_hall <: iHallActualSpeed;   iHallActualSpeed &= 0x00FFFFFF; }
			 else if  (cmd == 3) { c_hall <: iPosAbsolut;  		}
			 else if  (cmd == 4) { c_hall <: iHallError;   		}
			 else if  (cmd == 5) { c_hall <: iHallStateNew;   		}
			 else if  (cmd == 6) { c_hall <: iEncoderPinState;  }
			 else if  (cmd == 7) { c_hall <: iPosEncoder;   	}
			break;
			default:
			  break;
 	    }// end of select

  }// end while 1
}





