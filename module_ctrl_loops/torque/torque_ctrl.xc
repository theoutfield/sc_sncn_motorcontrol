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
#include<print.h>
#ifdef new
#define defParAngleUser 560
#define filter_length 30
#define filter_dc 160

//output is iUmotResult & iAnglePWM

torq_par t_param;
field_par f_param;
loop_par l_param;
int flag_field = 0;


void init_params_struct_all(torq_par &tor, field_par &field, loop_par &loop)
{
	tor.Kp_n = Torque_Kp_n;
	tor.Kp_d = Torque_Kp_d;
	tor.Ki_n = Torque_Ki_n;
	tor.Ki_d = Torque_Ki_d;
	tor.Integral_limit = Torque_Integral_limit;

	tor.Max_torque = Max_torque_out;

	field.Kp_n = Field_Kp_n;
	field.Kp_d = Field_Kp_d;
	field.Ki_n = Field_Ki_n;
	field.Ki_d = Field_Ki_d ;
	field.Integral_limit = Field_Integral_limit;

	loop.delay = loop_timing;

	return;
}

void init_torque_pars(torq_par &d)
{
	t_param.Kp_n = d.Kp_n;
	t_param.Kp_d = d.Kp_d;
	t_param.Ki_n = d.Ki_n;
	t_param.Ki_d = d.Ki_d;
	t_param.Integral_limit = d.Integral_limit;
	t_param.Max_torque = d.Max_torque;
}
void init_field_pars(field_par &d) //optional
{
	f_param.Kp_n = d.Kp_n;
	f_param.Kp_d = d.Kp_d;
	f_param.Ki_n = d.Ki_n;
	f_param.Ki_d = d.Ki_d;
	f_param.Integral_limit = d.Integral_limit;
	flag_field = 1;
}
void init_loop_pars(loop_par &d)
{
	l_param.delay = d.delay;
}
void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1, chanend c_value)
{
	int iAngleFromHallOld=0, iAnglePWM, iAnglePWMFromHall, iAnglePWMFromFOC;
	int iTemp; int a1=0, a2=0; int sinx, cosx, iTorqueSet = 0, iAlpha, iBeta, a1SquareMean=0, a2SquareMean=0, a1RMS, a2RMS;
	int VsdRef1, VsqRef1, VsdRef2, VsqRef2;		// invers park
	int VsaRef, VsbRef, iAngleInvPark, iId, iIq;
	int iActualSpeed, iAngleFromHall, iUmot = 0; int iSpeedValueNew;
	int iPhase1Sum=0, iPhase2Sum=0, iPhase1=0, iPhase2=0, iCountRMS =0, iTriggerRMS=0, ia1RMSMax;	unsigned a1Square=0,a2Square=0;
	int iCountDivFactor, iVectorInvPark, cmd; unsigned iVectorCurrent; int iFieldSet = 0;
	int value_0[filter_length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, value_1[filter_length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int value_d[filter_dc]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, value_q[filter_dc]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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

	if(flag_field == 0)
	{
		f_param.Kp_n = 25;
		f_param.Kp_d = 10;
		f_param.Ki_n = 2;
		f_param.Ki_d = 100;
		f_param.Integral_limit = 10000;
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

			iActualSpeed     =  get_hall_speed(c_hall_1);
			iAngleFromHall   = 	get_hall_angle(c_hall_1) & 0x0FFF;

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
#endif
