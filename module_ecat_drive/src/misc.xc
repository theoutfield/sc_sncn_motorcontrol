
/**
 * \file misc.xc
 * \author Pavan Kanajar <pkanajar@synapticon.com>
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


#include <refclk.h>
#include <qei_client.h>
#include <hall_client.h>
#include <comm_loop_client.h>
#include <drive_config.h>
#include <qei_config.h>

int detect_sensor_placement(chanend c_hall, chanend c_qei, chanend c_commutation)
{
	int times = 50;
	int valid;
	int current_pos_q;
	int current_pos_h;
	int hall_di;
	int avg_q = 0;
	int sum_h = 0;
	int difference_h;
	int difference_q;
	int previous_position_q = 0;
	int previous_position_h = 0;
	timer t;
	unsigned int time;
	int i;
	int sensor_placement_type; //1 in phase -1 out of phase
	int init_state;

	while(1)
	{
		init_state = __check_commutation_init(c_commutation);
		if(init_state == INIT)
		{

			printstrln("commutation intialized");
			init_state = INIT_BUSY;
			break;
		}
	}

	set_commutation_sinusoidal(c_commutation, 400);
	t :> time;

	while(sum_h ==0)
	{
		for(i = 0; i<times ; i++)
		{
			t when timerafter(time+100000) :> time;
			{current_pos_h, hall_di} = get_hall_position_absolute(c_hall);
			difference_h = current_pos_h - previous_position_h;
			sum_h = sum_h + difference_h;
			previous_position_h = current_pos_h;
		}
	}

	t :> time;
	while(avg_q == 0)
	{
		for(i = 0; i<times ; i++)
		{
			t when timerafter(time+3000) :> time;
			{current_pos_q, valid} = get_qei_position_absolute(c_qei);
			difference_q = current_pos_q - previous_position_q;
			if(difference_h > 10)
				difference_h = 0;
			else if(difference_h < -10)
				difference_h = 0;
			avg_q = avg_q + difference_q;
			previous_position_q = current_pos_q;
		}
	}
	//avg_q = 0;
	printintln(sum_h);
	printintln(avg_q);
	if(avg_q > 0 && sum_h <0)
	{
		sensor_placement_type = INVERTED;
		printstrln("inverted case1");
	}
	if(avg_q<0  && sum_h>0)
	{
		sensor_placement_type = INVERTED;
		printstrln("inverted case2");
	}
	if(avg_q>0 && sum_h>0)
	{
		sensor_placement_type = NORMAL;
		printstrln("normal case1");
	}
	if(avg_q<0 && sum_h<0)
	{
		sensor_placement_type = NORMAL;
		printstrln("normal case2");
	}
	set_commutation_sinusoidal(c_commutation ,0);
	return sensor_placement_type;
}
