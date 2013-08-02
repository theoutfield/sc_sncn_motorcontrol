#include "hall_server.h"
#include <stdlib.h>
#include <print.h>
#include <stdint.h>
#include "refclk.h"
#include "dc_motor_config.h"
#include <internal_config.h>
#include <xscope.h>

void run_hall(port in p_hall, hall_par &hall_params, chanend c_hall_p1,
		chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4)
{
	timer tx;
	unsigned int ts;
	unsigned command;

	unsigned angle1;			 // newest angle (base angle on hall state transition)
	unsigned delta_angle;
	unsigned angle;

	unsigned iCountMicroSeconds;
	unsigned iPeriodMicroSeconds;
	unsigned iTimeCountOneTransition = 0;
	unsigned iTimeSaveOneTransition = 0;

	unsigned pin_state; 		// newest hall state
	unsigned pin_state_last;
	unsigned new1, new2;
	unsigned uHallNext, uHallPrevious;
	int xreadings = 0;

	int iHallError = 0;
	int direction = 0;

	int position = 0;
	int previous_position = 0;
	int count = 0;
	int first = 1;
	int hall_enc_count = hall_params.pole_pairs * hall_params.gear_ratio * 4095;
	int time_elapsed; 			//between two transitions to calculate speed
	int init_state = INIT;

	tx	:> ts;
	while(1)
	{
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

		iCountMicroSeconds++; // period in usec
		iTimeCountOneTransition++;

		if(pin_state != pin_state_last)
		{
			if(pin_state == uHallNext)
			{
				direction = 1;
			}
			if(pin_state == uHallPrevious)
			{
				direction =-1;
			}

			//if(direction >= 0) // CW  3 2 6 4 5 1

			switch(pin_state)
			{
				case 3: angle1 = 0; uHallNext=2; uHallPrevious=1; break;
				case 2: angle1 = 682; uHallNext=6; uHallPrevious=3; break; //  60
				case 6: angle1 = 1365; uHallNext=4; uHallPrevious=2; break;
				case 4: angle1 = 2048; uHallNext=5; uHallPrevious=6; break; // 180
				case 5: angle1 = 2730; uHallNext=1; uHallPrevious=4; break;
				case 1: angle1 = 3413; uHallNext=3; uHallPrevious=5; break; // 300 degree
				default: iHallError++; break;
			}

			if(direction == 1)
			if(pin_state_last==1 && pin_state==3) // transition to NULL
			{
				iPeriodMicroSeconds = iCountMicroSeconds;
				iCountMicroSeconds = 0;
				if(iPeriodMicroSeconds)
				{
					time_elapsed = iPeriodMicroSeconds;
				}
			}

			if(direction == -1)
			if(pin_state_last==3 && pin_state==1)
			{
				iPeriodMicroSeconds = iCountMicroSeconds;
				iCountMicroSeconds = 0;
				if(iPeriodMicroSeconds)
				{
					time_elapsed = 0 - iPeriodMicroSeconds;
				}
			}

			iTimeSaveOneTransition = iTimeCountOneTransition;
			iTimeCountOneTransition = 0;
			delta_angle = 0;
			pin_state_last = pin_state;

		}// end (pin_state != pin_state_last


		#define defPeriodMax 1000000  //1000msec
		if(iCountMicroSeconds > defPeriodMax)
		{
			iCountMicroSeconds = defPeriodMax;
		}

		if(iTimeSaveOneTransition)
			delta_angle = (682 *iTimeCountOneTransition)/iTimeSaveOneTransition;
		if(delta_angle >= 680)
			delta_angle = 680;

		if(iTimeCountOneTransition > 50000)
			direction = 0;

		angle = angle1;
		if(direction == 1)
			angle += delta_angle;

		if(direction == -1)
			angle -= delta_angle;

		angle &= 0x0FFF; // 4095

		if(first == 1)
		{
			previous_position = angle;
			first =0;
		}

		if( previous_position != angle)
		{

			position = angle;
			if(position - previous_position <= -1800)
			{
				count = count + (4095 + position - previous_position);

			}

			else if(position - previous_position >= 1800)
			{
				count = count + (-4095 + position - previous_position);
			}
			else
			{
				count = count + position - previous_position;
			}
			previous_position = angle;

		}

		if(count > hall_enc_count || count < -hall_enc_count)
		{
			count = 0;
		}

		#pragma ordered
		select {
			case c_hall_p1 :> command:
				if (command == 1) {c_hall_p1 <: angle;}
				else if (command == 2) {c_hall_p1 <: time_elapsed;}
				else if (command == 3) {c_hall_p1 <: count;}
				break;

			case c_hall_p2 :> command:
				if (command == 1) {c_hall_p2 <: angle;}
				else if (command == 2) {c_hall_p2 <: time_elapsed;}
				else if (command == 3) {c_hall_p2 <: count;}
				break;

			case c_hall_p3 :> command:
				if (command == 1) {c_hall_p3 <: angle;}
				else if (command == 2) {c_hall_p3 <: time_elapsed;}
				else if (command == 3) {c_hall_p3 <: count;}
				break;

			case c_hall_p4 :> command:
				if (command == 1) {c_hall_p4 <: angle;}
				else if (command == 2) {c_hall_p4 <: time_elapsed;}
				else if (command == 3) {c_hall_p4 <: count;}
				else if (command == CHECK_BUSY) {c_hall_p4 <: init_state;}
				break;

			default:
				break;
		}

		tx when timerafter(ts + 250) :> ts;
	}
}

