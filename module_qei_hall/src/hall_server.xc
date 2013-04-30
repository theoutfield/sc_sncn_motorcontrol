/**
 * Module:  module_hall
 * Ludwig Orgler orgler@tin.it synapticon 04/2013
  *
 **/                                   
#include "hall_server.h"
#include <stdlib.h>
#include <stdint.h>
#include "refclk.h"
#include "dc_motor_config.h"



#define defParPolePairs                8
#define defParEncoderResolution     4000
#define defParEncoderZeroPoint       484



void run_encoder( chanend c_encoder,  port in p_encoder)
{
  timer tx;
  unsigned ts;
  unsigned cmd;

#define defEncoderPeriodMax	  100000  		// 200msec



  //=========== encoder =======================
  int iEncoderDirection 		=   0;
  int iCountEncoderPulses		=   0;
  int iNrEncoderPulses			= 500;
  int iEncoderCountMicroSeconds	=   0;
  int iEncoderPeriodMicroSeconds=   defEncoderPeriodMax;
  int iEncoderPeriodNext		=   defEncoderPeriodMax;
  int iEncoderSpeed				=   0;
  int iEncoderPeriodeRef		=   0;
  int iEncoderCountPeriodeRef	=   0;
  int iEncoderTimeOneTransition	=   0;
  int iEncoderCountOneTransition=   0;
  int iEncoderSpeedIsNew        =   0;

  int   iEncoderCalcFlag=0;
  int   iEncoderDividend;
  int   iEncState1	=0;
  int   iEncState2	=0;
  int   iEncOk;

  int iEncoderPinState;
  int iEncoderReferenz;
  int iEncoderStateNew;
  int iEncoderStateOld;
  int iEncoderError		    =0;
  int iEncoderPosAbsolut	=0;

  int iEncoderNext=0;
  int iEncoderPrevious=0;
  int iEncoderAngle1 = 0;
  int iEncoderAngle2 = 0;

  tx :> ts;  							// first value
  p_encoder :> iEncoderPinState;
  iEncoderStateNew = iEncoderPinState & 0x03;
  iEncoderStateOld = iEncoderStateNew;

  iEncoderDividend = 60000000/2;
  iEncoderDividend /= defParPolePairs;

#define defState0  0
#define defState1  2
#define defState2  3
#define defState3  1

//************************************encoder LOOP 2탎ec **************************************************
  	  while(1) {

	  iEncoderCountMicroSeconds++;   		// period in 탎ec
      iEncoderCountPeriodeRef++;
      iEncoderCountOneTransition++;

    //  p_ifm_ext_d2 <: 1;
      iEncOk  = 0;
      p_encoder :> iEncState1;
      iEncState1 &= 0x07;
      if(iEncState1 != iEncState2)
      {
    	  while(1)
    	  {
    	  p_encoder :> iEncState2;
    	  iEncState2 &= 0x07;
          if(iEncState1 == iEncState2)   iEncOk++;
          else {iEncState1 = iEncState2; iEncOk=0; }
          if(iEncOk > 4) break;
          }
      }
      if(iEncOk > 4 ) iEncoderPinState = iEncState2;
     // p_ifm_ext_d2 <: 0;


	  	  if(iEncoderCalcFlag)
    	  {
    	  iEncoderCalcFlag = 0;
    	  iEncoderSpeed    = 0;
    	  if(iEncoderPeriodMicroSeconds)
    	  {
    	  iEncoderSpeed = iEncoderDividend / iEncoderPeriodMicroSeconds;    // period in 탎ec
    	  }
    	  if(iEncoderDirection < 0) iEncoderSpeed = -iEncoderSpeed;
    	  iEncoderSpeedIsNew = 1;

    	  iEncoderPeriodNext = iEncoderPeriodMicroSeconds * 256;      // add 10%
    	  iEncoderPeriodNext /= 230;
    	  }

	 iEncoderStateNew = iEncoderPinState & 0x03;

	 if(iEncoderStateOld != iEncoderStateNew)
	 {
	     iEncoderStateOld = iEncoderStateNew;

		 iEncoderReferenz = iEncoderPinState & 0x04;
		 iEncoderStateNew = iEncoderPinState & 0x03;

		 if(iEncoderStateNew == iEncoderNext)    {iEncoderPosAbsolut++; iEncoderAngle1++; iEncoderDirection++; if(iEncoderDirection > +3) iEncoderDirection=+3;  }
	     if(iEncoderStateNew == iEncoderPrevious){iEncoderPosAbsolut--; iEncoderAngle1--; iEncoderDirection--; if(iEncoderDirection < -3) iEncoderDirection=-3;  }


	     if(iEncoderAngle1 < 0) iEncoderAngle1 = defParEncoderResolution + iEncoderAngle1;
	     if(iEncoderAngle1 >= defParEncoderResolution) iEncoderAngle1 -= defParEncoderResolution;

         if(iEncoderReferenz)
         {
        	 iEncoderAngle1			= defParEncoderZeroPoint;
         	 iEncoderPeriodeRef 	= iEncoderCountPeriodeRef;
        	 iEncoderCountPeriodeRef= 0;
         }

 		iEncoderAngle2 =  iEncoderAngle1 * defParPolePairs;
 		iEncoderAngle2 *= 4096;
 		iEncoderAngle2 /= defParEncoderResolution;  	// encoder steps per revolution
 		iEncoderAngle2 &= 0x0FFF;                       // modulo

        iEncoderTimeOneTransition  = iEncoderCountOneTransition;
        iEncoderCountOneTransition = 0;

	    switch(iEncoderStateNew)
	  	{
		  case defState0:   iEncoderNext=defState1; iEncoderPrevious=defState3;  break;
		  case defState1:   iEncoderNext=defState2; iEncoderPrevious=defState0;  break;
		  case defState2:   iEncoderNext=defState3; iEncoderPrevious=defState1;  break;
		  case defState3:   iEncoderNext=defState0; iEncoderPrevious=defState2;  break;
		  default: iEncoderError++; break;
	  	}

	      iCountEncoderPulses++;
	      if(iCountEncoderPulses >= iNrEncoderPulses)
	    	  {
	    	  iEncoderPeriodMicroSeconds     = iEncoderCountMicroSeconds;
	    	  iEncoderCalcFlag 				 = 1;
	    	  iCountEncoderPulses	         = 0;
	    	  iEncoderCountMicroSeconds      = 0;
	    	  }// end if(iCountEncoderPulses >= iNrEncoderPulses)
       }// ======== end of NewState ===================================


		if(iEncoderCountMicroSeconds > defEncoderPeriodMax)
		{
		iEncoderCountMicroSeconds = defEncoderPeriodMax;
		iEncoderSpeed      = 0;
		iEncoderSpeedIsNew = 1;
		}
		else
		{
		 if(iEncoderCountMicroSeconds > iEncoderPeriodNext)
		 {
		 iEncoderPeriodMicroSeconds  = iEncoderCountMicroSeconds;
    	 iEncoderCalcFlag = 1;
		 }
		}
//========================================== end encoder ===============================================



   tx when timerafter(ts + 500) :> ts;   // 250 => 1탎ec



#pragma ordered   // readout of 4 values about 700nsec
	    select {
		case c_encoder :> cmd:
		    if  (cmd == 1)
		    master
		    {
		 	c_encoder <: iEncoderSpeed;
			c_encoder <: iEncoderAngle2;
			c_encoder <: iEncoderPosAbsolut;
			c_encoder <: iEncoderPinState + iEncoderSpeedIsNew*256 + iEncoderReferenz * 65536;
			iEncoderSpeedIsNew  =    0;
			iEncoderPinState   &= 0xFF;
		    }



		break;
		default:  break;
	    }// end of select
  }// end while 1
}


void run_hall( chanend c_hall, port in p_hall)
 {
  timer tx;
  unsigned ts;					// newest timestamp
  unsigned cmd;
  int iTemp;
#define defHallPeriodMax	  100000  		// 200msec



  //============================================
  int iHallDirection          	= 0;
  int iCountHallPulses  	  	= 0;
  int iNrHallPulses     	  	= 2;
  int iHallCountMicroSeconds  	= 0;
  int iHallPeriodMicroSeconds 	= 0;
  int iHallPeriodNext		  	=  defHallPeriodMax;
  int iHallSpeed			 	= 0;
  int iHallSpeedTemp;
  int iHallSpeedIsNew          	=0;
  int iHallPeriodeRef		   	=0;
  int iHallCountPeriodeRef	    =0;


  int iHallPulsesLast=0;
  int iHallDividend;
  int iHallDividend2;

  int iHallCountTransitionSum;
  int iHallCountTransitionNew;
  int iHallCountTransitionFiltered;
  int iHallCountTransitionEstimated;
  int iHallCountOneTransition=0;

  unsigned iHallStateOld;
  unsigned iHallState1,iHallState2;
  unsigned uHallNext,uHallPrevious;

  int iStepHall      		= 0;
  int iHallError			= 0;
  int iHallAngleDeltaSum 	= 0;
  int iHallAngleDeltaValue  = 682;
  int iHallAngle1;		        // newest angle (base angle on hall state transition)
  int iHallOffset;

  int delta_angle;
  unsigned iHallStateNew;			// newest hall state
  int iHallAngle2			    = 0;
  int iHallTimeSaveOneTransition= 0;
  int iHallPosAbsolut		    = 0;
  int iHallStatusMachine        = 0;

  tx :> ts;  // first value

  p_hall :> iHallState1;
  iHallStateNew = iHallState1;
  iHallStateOld = -1;


  iHallDividend = 60000000/2;
  iHallDividend /= defParPolePairs;


//******************************************** LOOP 1탎ec **************************************************
  while(1) {

//============================================ H A L L =================================================
#define defHallState0 1
#define defHallState1 3
#define defHallState2 2
#define defHallState3 6
#define defHallState4 4
#define defHallState5 5


#define defPeriodMax 100000  //200msec

	  iHallCountMicroSeconds++;   		// period in 탎ec
	  iHallCountOneTransition++;
	  iHallCountPeriodeRef++;
	  iHallAngleDeltaSum += iHallAngleDeltaValue;


	  switch(iStepHall)
	  {
		  case 0: p_hall :> iHallState1; iHallState1 &= 0x07; iStepHall++;
		  	  	  break;
		  case 1: p_hall :> iHallState2; iHallState2 &= 0x07;
				  if(iHallState2 == iHallState1) iStepHall++;
				  else iStepHall=0;
				  break;
		  case 2: p_hall :> iHallState2; iHallState2 &= 0x07;
				  if(iHallState2 == iHallState1) iHallStateNew = iHallState2;
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
	  case 2: iHallSpeedTemp   = 0;
	  	  	  if(iHallPeriodMicroSeconds)
	  	  	  {
	  	  	  iHallSpeedTemp = iHallDividend2 / iHallPeriodMicroSeconds;  // period in 탎ec
	  	  	  }
	  	  	  iHallStatusMachine++;
		      break;
	  case 3: if(iHallDirection < 0) iHallSpeedTemp = -iHallSpeedTemp;
	          iHallSpeedIsNew = 1;
	          iHallSpeed = iHallSpeedTemp;
	  	  	  iHallStatusMachine++;
	  	  	  break;
	  case 4: iHallPeriodNext = iHallPeriodMicroSeconds * 256;    // add 10%
		  	  iHallStatusMachine++;
		      break;
	  case 5: iHallPeriodNext /= 230;
  	  	  	  iHallStatusMachine++;
		  	  break;
	  case 6: break;
	  }


      if(iHallStateNew != iHallStateOld)
      {
    	  	  iHallStateOld    = iHallStateNew;

 	      	  if(iHallStateNew == uHallNext)    { iHallPosAbsolut++; iHallDirection=+1;  iHallOffset=0;    }
 	      	  if(iHallStateNew == uHallPrevious){ iHallPosAbsolut--; iHallDirection=-1;  iHallOffset=682;  }

 	      	  switch(iHallStateNew)
 	 	      	  {
 				  case defHallState0: iHallAngle1 =     3776; 	 uHallNext=defHallState1; uHallPrevious=defHallState5;
 				  	  	  	  	  	  iHallPeriodeRef		= iHallCountPeriodeRef;
 				  	  	  	  	  	  iHallCountPeriodeRef	= 0;
 				  	  	  	  	  	  break;
 				  case defHallState1: iHallAngle1 =   363;  uHallNext=defHallState2; uHallPrevious=defHallState0;  break;   //  60
 				  case defHallState2: iHallAngle1 =  1046;  uHallNext=defHallState3; uHallPrevious=defHallState1;  break;
 				  case defHallState3: iHallAngle1 =  1728;  uHallNext=defHallState4; uHallPrevious=defHallState2;  break;   // 180
 				  case defHallState4: iHallAngle1 =  2411;  uHallNext=defHallState5; uHallPrevious=defHallState3;  break;
 				  case defHallState5: iHallAngle1 =  3094;  uHallNext=defHallState0; uHallPrevious=defHallState4;  break;   // 300 degree
 				  default: iHallError++; break;
 	 	      	  }// end switch

                iHallAngle1 += iHallOffset;
                iHallAngle1 &= 0x0FFF;
 	      	  //if(iHallDirection >= 0) // CW  3 2 6 4 5 1

	      iCountHallPulses++;
	      if(iCountHallPulses >= iNrHallPulses)
	      {
			  iHallPeriodMicroSeconds     = iHallCountMicroSeconds;
			  iHallCountMicroSeconds      = 0;
			  iHallPulsesLast 			  = iNrHallPulses;
			  iHallStatusMachine          = 1;
	    	  iCountHallPulses 		      = 0;

	    	  switch(iNrHallPulses)
	    		  {
	    		  case 2:  if(iHallPeriodeRef < 9000)    iNrHallPulses = 6;
	    	  	  	       break;
	    		  case 6:  if(iHallPeriodeRef > 12000)    iNrHallPulses = 2;
	    	  	           break;
	    		  default: iNrHallPulses=2; break;
	    		  }
	      }

	    	  iHallTimeSaveOneTransition  = iHallCountOneTransition;
	    	  iHallCountTransitionNew     = iHallCountOneTransition;
	    	  iHallCountOneTransition 	  = 0;
	    	  delta_angle             	  = 0;
      }//====================== end (iHallStateNew != iHallStateOld===========================


	 if(iHallCountMicroSeconds > defPeriodMax)
	 {
		 iHallCountMicroSeconds = defPeriodMax;
		 iHallSpeed             = 0;
		 iHallSpeedIsNew        = 1;
		 iNrHallPulses          = 2;
	 }
	 else
	 if(iHallCountMicroSeconds > iHallPeriodNext)
	     {
			  iHallPeriodMicroSeconds     = iHallCountMicroSeconds;
			  iHallPulsesLast 			  = iNrHallPulses;
			  iHallStatusMachine          = 1;
	      }


 		if(iHallCountOneTransition)
 		{
 		if(iHallCountOneTransition == 1)
 		{
 			iHallCountTransitionSum      	 -=  iHallCountTransitionFiltered;
 			iHallCountTransitionSum      	 +=  iHallCountTransitionNew;
 			iHallCountTransitionFiltered  	  =  iHallCountTransitionSum/4;
 			iHallCountTransitionEstimated     =  (iHallCountTransitionFiltered*3)/4 + iHallCountTransitionNew/4;

 			iTemp = iHallCountTransitionNew - iHallCountTransitionFiltered;
 			iTemp *= 682;
 			if(iHallCountTransitionFiltered)	iTemp /= iHallCountTransitionFiltered;
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
 		if(iHallDirection > 0)  iHallAngle2 += delta_angle;
 		if(iHallDirection < 0)  iHallAngle2 -= delta_angle;
 		iHallAngle2 &= 0x0FFF;

//======================end HALL ===================================================================



   tx when timerafter(ts + 500) :> ts;   // 250 => 1탎ec


	#pragma ordered   // readout of 4 values about 700nsec
 	    select {
			case c_hall :> cmd:
			    if  (cmd == 1)
			    master
			    {   	c_hall <: iHallSpeed;
			 	 	 	c_hall <: iHallAngle2;
			 	 	 	c_hall <: iHallPosAbsolut;
			 	 	 	c_hall <: iHallStateNew + iHallSpeedIsNew * 256;
			 	 	 	          iHallSpeedIsNew  =  0;
			    }

			break;
			default:  break;
 	    }// end of select
  }// end while 1
}






