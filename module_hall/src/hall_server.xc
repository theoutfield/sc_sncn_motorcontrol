
/**
 * \file hall_server.xc
 * \brief Hall Sensor Server Implementation
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "hall_server.h"
//#pragma xta command "analyze loop hall_loop"
//#pragma xta command "set required - 10.0 us"
//#define DEBUG

void hall_client_handler(chanend c_hall, int command, int angle, int raw_velocity, int init_state,\
		int &count, int direction, hall_par &hall_params, int &status)
{
	switch(command)
	{
		case HALL_POS_REQ:
			c_hall <: angle;
			break;

		case HALL_VELOCITY_REQ:
			c_hall <: raw_velocity;
			break;

		case HALL_ABSOLUTE_POS_REQ:
			c_hall <: count;
			c_hall <: direction;
			break;

		case CHECK_BUSY:
			c_hall <: init_state;
			break;

		case SET_HALL_PARAM_ECAT:
			c_hall :> hall_params.pole_pairs;
			c_hall :> hall_params.max_ticks;
			c_hall :> hall_params.max_ticks_per_turn;
			status = 1;
			break;

		case RESET_HALL_COUNT:
			c_hall :> count;
			break;

		default:
			break;
	}
}

void run_hall(chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4, \
		chanend c_hall_p5, chanend c_hall_p6, port in p_hall, hall_par &hall_params)
{
	timer tx;
	unsigned int ts;
	unsigned int command;

	unsigned int angle1 = 0;			 // newest angle (base angle on hall state transition)
	unsigned int delta_angle = 0;
	unsigned int angle = 0;

	unsigned int iCountMicroSeconds = 0;
	unsigned int iPeriodMicroSeconds = 0;
	unsigned int iTimeCountOneTransition = 0;
	unsigned int iTimeSaveOneTransition = 0;

	unsigned int pin_state = 0; 		// newest hall state
	unsigned int pin_state_last = 0;
	unsigned int new1 = 0;
	unsigned int new2 = 0;
	unsigned int uHallNext = 0;
	unsigned int uHallPrevious = 0;
	int xreadings = 0;

	int iHallError = 0;
	int direction = 0;

	int position = 0;
	int previous_position = 0;
	int count = 0;
	int first = 1;
	int hall_max_count = hall_params.max_ticks;
	int time_elapsed = 0;
	int init_state = INIT;

	timer t1;
	int time1;
	int init_velocity = 0;
	int position1 = 0;
	int previous_position1 = 0;
	int velocity = 0;
	int difference1 = 0;
	int old_difference = 0;
	int filter_length = FILTER_LENGTH_HALL;
	int filter_buffer[FILTER_LENGTH_HALL];
	int index = 0;
	int raw_velocity = 0;
	int hall_crossover = (4096 * 9 )/10;
	int status = 0; //1 changed

	init_filter(filter_buffer, index, filter_length);
#ifdef DEBUG
	{
		xscope_register(3, XSCOPE_CONTINUOUS, "0 hall_position", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 hall_velocity", XSCOPE_INT,	"n",
				           XSCOPE_CONTINUOUS, "1 hall_velocity", XSCOPE_INT,	"n");
		xscope_config_io(XSCOPE_IO_BASIC);
	}
#endif
	/* Init hall sensor */
	p_hall :> pin_state;
	switch(pin_state)
	{
		case 3: angle = 0;
				break;
		case 2: angle = 682;
				break; //  60
		case 6: angle = 1365;
				break;
		case 4: angle = 2048;
				break; // 180
		case 5: angle = 2730;
				break;
		case 1: angle = 3413;
				break; // 300 degree
	}

	t1 :> time1;
	tx :> ts;
	while(1)
	{
//#pragma xta endpoint "hall_loop"
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

		iCountMicroSeconds = iCountMicroSeconds + 10; // period in 10 usec
		iTimeCountOneTransition = iTimeCountOneTransition + 10 ;

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

		if(count > hall_max_count || count < -hall_max_count)
		{
			count = 0;
		}

		#ifdef DEBUG
			xscope_probe_data(0, angle);
			xscope_probe_data(1, count);
			xscope_probe_data(2, raw_velocity);
		#endif


		#pragma ordered
		select {
			case c_hall_p1 :> command:
				hall_client_handler(c_hall_p1, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case c_hall_p2 :> command:
				hall_client_handler(c_hall_p2, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case c_hall_p3 :> command:
				hall_client_handler(c_hall_p3, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case c_hall_p4 :> command:
				hall_client_handler(c_hall_p4, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case c_hall_p5 :> command:
				hall_client_handler(c_hall_p5, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case c_hall_p6 :> command:
				hall_client_handler(c_hall_p6, command, angle, raw_velocity, init_state, count, \
						direction, hall_params, status);
				break;

			case tx when timerafter(time1 + MSEC_FAST) :> time1:
					if(init_velocity == 0)
					{
						//position1 = count;
						if(count > 2049)
						{
							init_velocity = 1;
							previous_position1 = 2049;
						}
						else if(count < -2049)
						{
							init_velocity = 1;
							previous_position1 = -2049;
						}
						velocity = 0;
					}
					else
					{
						difference1 = count - previous_position1;
						if(difference1 > hall_crossover)
							difference1 = old_difference;
						else if(difference1 < -hall_crossover)
							difference1 = old_difference;
						velocity = difference1;
						#ifdef Debug_velocity_ctrl
								xscope_probe_data(0, velocity);
						#endif
						previous_position1 = count;
						old_difference = difference1;
					}
					raw_velocity = _modified_internal_filter(filter_buffer, index, filter_length, velocity);
				break;

			default:
				break;
		}
		if(status == 1)
		{
			hall_max_count = hall_params.max_ticks;
			status = 0;
		}

		tx when timerafter(ts + 2500) :> ts; //10 usec 2500

//#pragma xta endpoint "hall_loop_stop"
	}
}

