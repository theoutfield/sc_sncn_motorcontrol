/**
 * \file torque_ctrl.xc
 *
 *	Torque control rountine based on field oriented Torque control method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include "torque_ctrl.h"
#include <refclk.h>
#include <xscope.h>
#include <print.h>
#include "hall-qei.h"
//#define ENABLE_xscope_torq
#define defParAngleUser 560
#define filter_length 30
#define filter_dc 160

unsigned root_function(unsigned uSquareValue);
int get_torque(chanend c_torque)
{
	int torque;
	c_torque <: 3;
	c_torque :> torque;
	return torque;
}

void set_torque(chanend c_torque, int torque)
{
	c_torque <: 2;
	c_torque <: torque;
	return;
}

void current_ctrl_loop(chanend sig, chanend signal2, chanend adc, chanend c_hall_1,
		chanend sync_output, chanend c_commutation,
		chanend c_torque) {
	int a1 = 0, a2 = 0;
	int iActualSpeed;
	int cmd;
	int value_0[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			value_1[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	timer ts , ts1;
	int time, time2;
	int fil_cnt = 0;
	int ia_f = 0, ib_f = 0;
	int i = 0;
	int fl = 30;
	int j = 0;
	int flc = 3;
	int mod;
	int prev_v = 0, dirn = 1;
	int mod_speed;
	int filter_count = 0;

	/* Torque control variables */
	int i_hall_angle;
	int sin_x, cos_x, iAlpha, iBeta;
	int iId, iIq;
	int iUmot = 0;
	int iPhase1 = 0, iPhase2 = 0;
	int value_d[filter_dc], value_q[filter_dc];
	timer tc;
	int time1;
	unsigned theta; // angle
	int i1 = 0, j1 = 0, mod1 = 0;

	int iq_fi = 0, id_fi = 0, fdc = filter_dc, fil_cnt_dc = 0;
	int fldc = filter_dc, Speed = 0;

	int wait = 21000;

	int torque_target = 0;
	int torque_actual;
	int torque_error = 0;
	int torque_error_integral = 0;
	int torque_error_derivative = 0;
	int torque_error_previous = 0;
	int torque_control_output = 0;
	const int TORQUE_INTEGRAL_MAX = 137000;
	int input_torque;

	int flag1= 0; int count_cur = 0;
	/* PID Controller variables */
	int Kp;							// Proportional gain
	int Ki;							// Integral gain
	int Kd;							// Derivative gain

	int proportional_member = 0;
	int integral_member = 0;
	int derivative_member = 0;

	int TORQUE_OUTPUT_MAX = 13739;
	Kp = 15; Kd = 1; Ki = 11;
	while (1) {
		unsigned found = 0;
		select
		{
			case sig :> cmd:
				do_adc_calibration_ad7949(adc); found =1;
				break;
			default:
				break;
		}
		if(found == 1)
		{
			break;
		}
	}
	sig <: 1;
	signal2 <: 1;
	ts :> time;
	tc :> time1;
	ts1 :> time2;

	while(1)
	{
	#pragma ordered
		select
		{
			case ts when timerafter(time+5556) :> time: // .05 ms
				{	a1 , a2}= get_adc_vals_calibrated_int16_ad7949(adc);
				value_0[fil_cnt] = a1;
				value_1[fil_cnt] = a2;

				fil_cnt = (fil_cnt+1)%fl;
				ia_f = 0; ib_f = 0;
				j=0;
				for(i=0; i<flc; i++)
				{
					mod = (fil_cnt - 1 - j)%fl;
					if(mod<0)
					mod = fl + mod;
					ia_f += value_0[mod];
					ib_f += value_1[mod];
					j++;
				}
				ia_f /= flc;
				ib_f /= flc;
			//	xscope_probe_data(0, ia_f);
			//	xscope_probe_data(1, ib_f);

				filter_count++;
				if(filter_count == 10)
				{
					filter_count = 0;
					iActualSpeed = get_speed_cal(c_hall_1);
				//	xscope_probe_data(0, iActualSpeed);
					mod_speed = iActualSpeed;
					if(iActualSpeed<0)
					mod_speed = 0 - iActualSpeed;
					if(mod_speed<370)
					{
						flc = 20;
					}
					if(mod_speed < 100)
					{
						flc = 50;
					}
					if(mod_speed>800)
					flc = 3;
				}
			break;


			case tc when timerafter(time1+wait) :> time1:
				wait = 21000;
				i_hall_angle = get_sync_position ( sync_output ); //synced input

				iPhase1 = 0 - ia_f;
				iPhase2 = 0 - ib_f;

				iAlpha = iPhase1;
				iBeta = (iPhase1 + 2*iPhase2); // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
				iBeta *= 37838;
				iBeta /= 65536;
				iBeta = -iBeta;

				/* ==== Park transform ==== */
				theta = i_hall_angle; //range 500 qei
				theta &= 499;
				sin_x = newsine_table[theta]; // sine( theta );
				theta = (125 - theta); // 90-theta
				theta &= 499; //499
				cos_x = newsine_table[theta]; // values from 0 to +/- 16384

				iId = (((iAlpha * cos_x ) /16384) + ((iBeta * sin_x ) /16384));
				iIq = (((iBeta * cos_x ) /16384) - ((iAlpha * sin_x ) /16384));

				value_d[fil_cnt_dc] = iId;
				value_q[fil_cnt_dc] = iIq;
				fil_cnt_dc = (fil_cnt_dc+1)%fdc;
				id_fi = 0; iq_fi = 0;

				j1=0;
				for(i1=0; i1<fldc; i1++)
				{
					mod1 = (fil_cnt_dc - 1 - j1)%fdc;
					if(mod1<0)
					mod1 = fdc + mod1;
					id_fi += value_d[mod1];
					iq_fi += value_q[mod1];
					j1++;
				}
				id_fi /= fldc;
				iq_fi /= fldc;

				torque_actual = root_function(iq_fi*iq_fi+id_fi*id_fi);

				#ifdef ENABLE_xscope_torq
				xscope_probe_data(0, torque_actual);
				xscope_probe_data(1, torque_target);
				#endif

				Speed = iActualSpeed;

				if(Speed<0)
					Speed = 0 -Speed;
				if(Speed<370)
				{
					fldc = 27; //40
				}

				else if( Speed > 370 && Speed <1000)
				{
					fldc = 27; //30
				}
				else
				{
					fldc = 20;
				}

				break;

			case ts1 when timerafter(time2+7700) :> time2:

				if(torque_actual > 400)
				{
					Kp = 15; Kd = 1; Ki = 11;

					torque_error = 350 - torque_actual;
					torque_error_integral = torque_error_integral + torque_error;
					torque_error_derivative = torque_error - torque_error_previous;

#ifdef ENABLE_xscope_torq
					//xscope_probe_data(2, torque_error);
#endif

					if(torque_error_integral > TORQUE_INTEGRAL_MAX)
					{
						torque_error_integral = TORQUE_INTEGRAL_MAX;
					}
					else if(torque_error_integral < 0)
					{
						torque_error_integral = 0 ;
					}

					if(torque_error_integral == 0) torque_error_integral = 1;

					proportional_member = (Kp * torque_error)/10;
					integral_member = (Ki * torque_error_integral)/110;
					derivative_member = (Kd * torque_error_derivative)/10;

					torque_control_output = proportional_member + integral_member + derivative_member;

#ifdef ENABLE_xscope_torq
					//xscope_probe_data(3, integral_member);
					//xscope_probe_data(4, torque_control_output);
#endif
					torque_error_previous = torque_error;


					if(torque_target >=0)
					{
						if(torque_control_output >= TORQUE_OUTPUT_MAX) {
							torque_control_output = TORQUE_OUTPUT_MAX;
						}
						else if(torque_control_output < 0){
							torque_control_output = 0;
						}
					}
					else
					{
						if(torque_control_output <= -TORQUE_OUTPUT_MAX) {
							torque_control_output = 0 - TORQUE_OUTPUT_MAX;
						}
						else if(torque_control_output > 0){
							torque_control_output = 0;
						}
					}


					flag1=1;
					c_commutation <: 2;
					c_commutation <: torque_control_output;
					torque_target = torque_control_output;
				}
				else
				{
					//if(flag1==0)
					{

flag1=0;
					c_commutation <: 2;
					c_commutation <: torque_target;
					torque_error_integral = (torque_target*110)/Ki;
					}
				}

				break;


		}

		select
		{
			case c_torque:> cmd:
				if(cmd == 2)
				{
					c_torque :> input_torque;
					if(input_torque>13739)
					{
						input_torque = 13739;
					}
					if(flag1 == 1)
					{
						count_cur = count_cur+1;
						if(count_cur>9004)
						{
							count_cur = 0;
							torque_target = input_torque;
						}
					}
					else if(flag1==0)
					{
						torque_target = input_torque;
					}

					//if(torque_target > 1000)  //range 13700
					//	torque_target = 1000;

				}
				if(cmd == 3)
				{
					c_torque <: torque_actual;
				}
				if(cmd==4)
				{
					c_torque<:iActualSpeed;
				}
				break;
			default:
				break;

		}
	}
}


