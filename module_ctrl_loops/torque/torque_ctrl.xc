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


//output is iUmotResult & iAnglePWM
void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall, chanend c_value, torq_par &t_param, field_par &f_param, loop_par &l_param)
{
	int iAngleFromHallOld=0, iAnglePWM, iAnglePWMFromHall, iAnglePWMFromFOC;
	int iTemp; int a1=0, a2=0; int sinx, cosx, iTorqueSet = 0, iAlpha, iBeta, a1SquareMean=0, a2SquareMean=0, a1RMS, a2RMS;
	int VsdRef1, VsqRef1, VsdRef2, VsqRef2;		// invers park
	int VsaRef, VsbRef, iAngleInvPark, iId, iIq;
	int iActualSpeed, iAngleFromHall, iUmot = 0; int iSpeedValueNew;
	int iPhase1Sum=0, iPhase2Sum=0, iPhase1=0, iPhase2=0, iCountRMS =0, iTriggerRMS=0, ia1RMSMax;	unsigned a1Square=0,a2Square=0;
	int iCountDivFactor, iVectorInvPark, cmd; unsigned iVectorCurrent; int iFieldSet = 0;
	int value_0[filter_length], value_1[filter_length];
	int value_d[filter_dc], value_q[filter_dc];
	timer ts; int time, time1;
	int t1, t2, t3, t4 , t5;
	int e_d = 0, e_di = 0;
	int e_q = 0, e_qi = 0;
	int iId_f=0, iIq_f=0;
	char cTriggerPeriod=0; int iMotHoldingTorque = 0;
	unsigned theta;  // angle
	int reached = 0;
	int Kd_q, e_qd, prev =0;
	int fil_cnt =0;
	int ia_f=0, ib_f=0; int i =0;int fl=filter_length; int j=0;
	int iq_fi=0, id_fi=0 , fdc = filter_dc, fil_cnt_dc = 0; int flc =filter_length; int mod, fldc = filter_dc ;
	VsdRef1 = 0; VsqRef1 = 0;


	for(i = 0; i < filter_length; i++)
	{
		value_0[i] = 0;
		value_1[i] = 0;
	}
	for(i = 0; i < filter_dc; i++)
	{
		value_d[i] = 0;
		value_q[i] = 0;
	}
	 while(1)
	 {
		  unsigned cmd, found = 0;
		  select
		  {
			case sig :> cmd:
				do_adc_calibration_ad7949(adc); found =1 ;
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

	ts :> time;
	ts :> t1;

	while(1)
	{
		do
		{
			{a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc);
			value_0[fil_cnt] = a1;
			value_1[fil_cnt] = a2;

			fil_cnt = (fil_cnt+1)%fl ;
			ia_f = 0; ib_f = 0;
			j=0;
			for(i=0 ; i<flc; i++)
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

			iActualSpeed     =  get_hall_speed(c_hall);
			iAngleFromHall   = 	get_hall_angle(c_hall) & 0x0FFF;

			iActualSpeed    &= 0x00FFFFFF;
			if(iActualSpeed & 0x00FF0000)
				iActualSpeed |= 0xFFFF0000;

			if(iActualSpeed<370)
			{	flc  = 20;
				fldc = 160;
			}
			if(iActualSpeed>=370)
				{flc  = 20;
				fldc = 160;}
			if(iActualSpeed>=700)
				{flc  = 5;
				fldc = 160;}

			if(iActualSpeed>=1120 && iActualSpeed<=1378)
			{	flc  = 2;
				fldc = 160;

			}
			if(iActualSpeed>=1400)
			{	flc  = 2;
				fldc = 80;

			}
			if(iActualSpeed>=2000)
			{	flc  = 2;
				fldc = 80;
			}
			ia_f=0-ia_f;
			ib_f=0-ib_f;
			if(iActualSpeed > 0)
			{
				if(iAngleFromHallOld > 2048  && iAngleFromHall < 2048)
				cTriggerPeriod = 0xFF;
			}
			else if(iActualSpeed < 0)
			{
				if(iAngleFromHallOld < 2048  && iAngleFromHall > 2048)
				cTriggerPeriod = 0x7F;
			}

			iAngleFromHallOld = iAngleFromHall;


			iPhase1   =   ia_f;
			iPhase2   =   ib_f;


			iCountRMS++;
			//=================== RMS =====================
			if(iCountRMS > 18000) iTriggerRMS = 5;

			if(cTriggerPeriod & 0x02)
			{
				 cTriggerPeriod &= 0x02^0xFF;
				 iTriggerRMS++;
			}

			if(iTriggerRMS > 1)
				if(iCountRMS)
				{
					iCountRMS = 0;
					if(iTriggerRMS >= 5) iTriggerRMS=0;
					else iTriggerRMS--;
				}

			iAlpha =  iPhase1;
			iBeta  =  (iPhase1 + 2*iPhase2);   		// iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
			iBeta *=  37838;
			iBeta /=  65536;
			iBeta  =  -iBeta;

			 //============= park transform ============================
			theta = iAngleFromHall/16;
			theta &= 0xFF;
			sinx  = sine_table[theta];      		// sine( theta );
			theta = (64 - theta);  		    		// 90-theta
			theta &= 0xFF;
			cosx  = sine_table[theta];      		// values from 0 to +/- 16384

			iId = (((iAlpha * cosx )  /16384) + ((iBeta * sinx ) /16384));
			iIq = (((iBeta  * cosx )  /16384) - ((iAlpha * sinx ) /16384));


			value_d[fil_cnt_dc] = iId;
			value_q[fil_cnt_dc] = iIq;
			fil_cnt_dc = (fil_cnt_dc+1)%fdc;
			id_fi  = 0; iq_fi = 0;

			j=0;
			for(i=0 ; i<fldc; i++)
			{
				mod = (fil_cnt_dc - 1 - j)%fdc;
				if(mod<0)
					mod = fdc + mod;
				id_fi += value_d[mod];
				iq_fi += value_q[mod];
				j++;
			}
			id_fi /= fldc;
			iq_fi /= fldc;


			select
			{
				case ts when timerafter(time+l_param.delay) :> time:
						{//PI loops for FOC defined in dc_motor_config

							//ts when timerafter(time+23333) :> void;												//50   10
							e_d = (iFieldSet  - id_fi);
							VsdRef2 = (f_param.Kp_n * e_d)/f_param.Kp_d + (e_di * f_param.Ki_n)/f_param.Ki_d;		//10
							e_di += e_d;


							//Kp_q = 40; Ki_q = 4; 								//Kd_q= 2; //20  12
							e_q = (iTorqueSet  -  iq_fi);
							VsqRef2 = (t_param.Kp_n * e_q)/t_param.Kp_d + (e_qi * t_param.Ki_n )/t_param.Ki_d  ;  			//10
							e_qi += e_q;

							//e_qd = e_q - prev;							//prev = e_q;


							if(e_qi > t_param.Integral_limit)
								e_qi = t_param.Integral_limit;
							else if(e_qi < -t_param.Integral_limit)
								e_qi = 0 - t_param.Integral_limit;

							if(e_di > f_param.Integral_limit)
								e_di = f_param.Integral_limit;
							else if(e_di < -f_param.Integral_limit)
								e_di = 0 - f_param.Integral_limit;



							if(VsqRef2 > 5000)
								VsqRef2  = 5000;
							if(VsdRef2 > 5000)
								VsdRef2  = 5000;


							if(VsqRef2 < -5000)
								VsqRef2  = -5000;
							if(VsdRef2 < -5000)
								VsdRef2  = -5000;

							 //================= invers park transformation =====================
							VsaRef = VsdRef2 * cosx/16384   - VsqRef2 * sinx/16384;
							VsbRef = VsdRef2 * sinx/16384   + VsqRef2 * cosx/16384;
							iAngleInvPark  = arctg1(VsaRef,VsbRef);           			// from 0 - 4095


							iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
							iVectorInvPark = root_function(iVectorInvPark);

							iVectorCurrent = iAlpha * iAlpha + iBeta * iBeta;
							iVectorCurrent = root_function(iVectorCurrent);


							//if(a1SquareMean)
								//a1RMS = root_function(a1SquareMean);
							//a1SquareMean = 0;
							//if(a2SquareMean)
								//a2RMS = root_function(a2SquareMean);
							//a2SquareMean = 0;

							iUmot = iVectorInvPark/2;
							if(iUmot > 4096)
								iUmot = 4096;


							{
								iAnglePWMFromHall = iAngleFromHall + defParAngleUser  ;
								iAnglePWMFromHall =  iAngleFromHall & 0x0FFF;
								iAnglePWMFromFOC  = iAngleInvPark + (4096 - 1020) + defParAngleUser ;
								iAnglePWM = iAnglePWMFromFOC & 0x0FFF;
							}

						   //================== Holding Torque if motor stopped ====================================================

							if(iMotHoldingTorque)
							{
								//if(iStep1 != 0) iAngleLast = iAnglePWM;
								//if(iStep1 == 0 )
								//iAnglePWM = iAngleLast; //  + iAngleFromHall  - 600;
							}
							iAnglePWM &= 0x0FFF;
							c_value <: 40;
							c_value <: iUmot;					//to PWM loop 18Khz
							c_value <: iAnglePWM;
							iId_f = 0; iIq_f = 0;
							ts :> t1;
						}
				     	reached = 1;
						ts:>t2;
					break;
				default:
						reached = 0;
					break;
			}

			reached = 0;
			//ts:>time;


			select
			{
				case input:> cmd:
					if(cmd == 20)				//user cmd token 20
					{
						input :> cmd;   		// from user or other external loops
						iTorqueSet = cmd;
						if(cmd > t_param.Max_torque)
							iTorqueSet = t_param.Max_torque;
						else if(cmd < -t_param.Max_torque)
							iTorqueSet = 0 - t_param.Max_torque;
					}
					break;
				default:
					break;

			}
		}while(!reached);

	}
	//output is iUmotResult & iAngleInvPark

}

