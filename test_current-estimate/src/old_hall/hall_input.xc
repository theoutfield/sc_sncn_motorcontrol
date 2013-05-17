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

void run_hall( chanend c_hall_p1, port in p_hall, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4)
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

  unsigned pin_state;			// newest hall state
  unsigned pin_state_last;
  unsigned new1,new2;
  unsigned uHallNext,uHallPrevious;
  int xreadings=0;
  int iPosAbsolut;

  int iHallError=0;
  int dir=0;


  unsigned uvalue, pos_cmd;
  signed spos, prev=0, count=0, co12=0,first=1,set=0;
  int hall_enc_count =  POLE_PAIRS * GEAR_RATIO * 4095;
  int speed_calc;
  tx :> ts;  // first value

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
				  if(new2 == new1) pin_state = new2;
				  else xreadings=0;
				  break;
	  }

	  iCountMicroSeconds++;   // period in µsec
	  iTimeCountOneTransition++;

      if(pin_state != pin_state_last)
      {
 	      if(pin_state == uHallNext)
 	      {
 	    	//  iPosAbsolut =  iPosAbsolut+60)%360;
 	    	  dir = 1;
 	      }
	      if(pin_state == uHallPrevious)
	      {
	    	 // iPosAbsolut = (iPosAbsolut-60)%360;
	    	  dir =-1;
	      }

	      //if(dir >= 0) // CW  3 2 6 4 5 1

	      switch(pin_state)
	  	  {
			  case 3: angle1 =     0;  uHallNext=2; uHallPrevious=1;  break;
			  case 2: angle1 =   682;  uHallNext=6; uHallPrevious=3;  break;   //  60
			  case 6: angle1 =  1365;  uHallNext=4; uHallPrevious=2;  break;
			  case 4: angle1 =  2048;  uHallNext=5; uHallPrevious=6;  break;   // 180
			  case 5: angle1 =  2730;  uHallNext=1; uHallPrevious=4;  break;
			  case 1: angle1 =  3413;  uHallNext=3; uHallPrevious=5;  break;   // 300 degree
			  default: iHallError++; break;
	  	  }


	    if(dir == 1)
	    	if(pin_state_last==1 && pin_state==3) // transition to NULL
	    	{
				 iPeriodMicroSeconds     = iCountMicroSeconds;
				 iCountMicroSeconds      = 0;
				 iHallActualSpeed        = 0;
	    		if(iPeriodMicroSeconds)
	    		{
	    			speed_calc = iPeriodMicroSeconds;
	    		//	iHallActualSpeed = (60000000/iPeriodMicroSeconds)/POLE_PAIRS;    // period in µsec
					//iHallActualSpeed /= POLE_PAIRS; //Pole pairs
	    		}
	    	}

	    if(dir == -1)
	    	if(pin_state_last==3 && pin_state==1)
	    	{
				iPeriodMicroSeconds     = iCountMicroSeconds;
				iCountMicroSeconds      = 0;
				iHallActualSpeed        = 0;
	    		if(iPeriodMicroSeconds)
	    		{
	    			speed_calc =0-iPeriodMicroSeconds;
	    		//	iHallActualSpeed = 0 - (60000000/iPeriodMicroSeconds)/POLE_PAIRS;
	    			//iHallActualSpeed = 60000000/iPeriodMicroSeconds;   	// transition 60 * 1000000 (sec multiplier) / time in Usec
					//iHallActualSpeed /= POLE_PAIRS;						// pole correction for speed
					//iHallActualSpeed = -iHallActualSpeed;
	    		}
	    	}

		   iTimeSaveOneTransition  = iTimeCountOneTransition;
		   iTimeCountOneTransition = 0;
		   delta_angle     = 0;
		   pin_state_last  = pin_state;

      }// end (pin_state != pin_state_last
     //===============================================================


	#define defPeriodMax 1000000  //1000msec
		if(iCountMicroSeconds > defPeriodMax)
			{
				iCountMicroSeconds = defPeriodMax;
				iHallActualSpeed=0;
			}


 		if(iTimeSaveOneTransition)
		delta_angle = (682 *iTimeCountOneTransition)/iTimeSaveOneTransition;
 		if(delta_angle >= 680) delta_angle = 680;

	  if(iTimeCountOneTransition > 50000) dir = 0;

	  angle2 = angle1;
      if(dir == 1)  angle2 += delta_angle;

      if(dir == -1) angle2 -= delta_angle;

      angle2 &= 0x0FFF;    // 4095

//	  tx :> ts;

 /*     if(first == 1)
      {
		prev = angle2;
		first =0;  set = prev;
      }

	if( prev != angle2)
	{

		spos=angle2;
		if(spos-prev <= -1800){

				count = count + (4095 + spos-prev);
			   // c=c+1;
		          //forw
		}

		else if(spos-prev >= 1800){
					count = count + (-4095 + spos-prev);
			   // c=c-1;
		            // revr
		}

		else{

			count = count + spos-prev;
		}
		prev=angle2;

	}

	if(count>hall_enc_count || count< -hall_enc_count)
	{
		//printint( count);printstrln(" one");
		count=0;
	}
*/


	//xscope_probe_data(1,iHallActualSpeed);
	#pragma ordered
 	    select {
			case c_hall_p1 :> cmd:

				  if  (cmd == 1) { c_hall_p1 <: angle2; }
			 else if  (cmd == 2) { c_hall_p1 <: speed_calc;  }
			 else if  (cmd == 3) { c_hall_p1 <: count;  }
			 else if  (cmd == 4) { c_hall_p1 <: iHallError;   }
			break;
			case c_hall_p2 :> cmd:

				  if  (cmd == 1) { c_hall_p2 <: angle2; }
			 else if  (cmd == 2) { c_hall_p2 <: speed_calc;  }
			 else if  (cmd == 3) { c_hall_p2 <: count;  }
			 else if  (cmd == 4) { c_hall_p2 <: iHallError;   }
			break;

			case c_hall_p3 :> cmd:
				  if  (cmd == 1) { c_hall_p3 <: angle2; }
			 else if  (cmd == 2) { c_hall_p3 <: speed_calc;  }
			 else if  (cmd == 3) { c_hall_p3 <: count;  }
			 else if  (cmd == 4) { c_hall_p3 <: iHallError;   }
			break;

			case c_hall_p4 :> cmd:
				  if  (cmd == 1) { c_hall_p4 <: angle2; }
			 else if  (cmd == 2) { c_hall_p4 <: speed_calc;  }
			 else if  (cmd == 3) { c_hall_p4 <: count;  }
			 else if  (cmd == 4) { c_hall_p4 <: iHallError;   }
			break;

			default:
			  break;
 	    }// end of select
 	   tx when timerafter(ts + 250) :> ts;
  }// end while 1
}





