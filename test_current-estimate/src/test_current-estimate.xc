/*
 *
 * \file
 * \brief Main project file
 *
 * Port declarations, etc.
 *
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 0.1 (2012-11-23 1850)
 *\Motor 3 motion profile size optimized code for position ctrl loops
*/



#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <stdio.h>
#include <stdint.h>
#include "ioports.h"
#include "hall-qei.h"
#include "hall_input.h"
#include "hall_client.h"
#include "pwm_service_inv.h"
#include "adc_ad7949.h"

#include "pwm_config.h"
#include "comm_sine.h"
#include "refclk.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include "sine_table_big.h"
#define COM_CORE 0
#define IFM_CORE 3



//on stdcore[3]: in           port    ADC_SYNC_PORT = XS1_PORT_16A;

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define filter_length 30
#define filter_dc 160
#define filter_ph 160
void filter_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1, chanend c_value, chanend sync_output)
{
	int iAngleFromHallOld=0, iAnglePWM, iAnglePWMFromHall, iAnglePWMFromFOC;
	int iTemp; int a1=0, a2=0; int sinx, cosx, iTorqueSet = 50, iAlpha, iBeta, a1SquareMean=0, a2SquareMean=0, a1RMS, a2RMS;
	int VsdRef1, VsqRef1, VsdRef2, VsqRef2;		// invers park
	int VsaRef, VsbRef, iAngleInvPark, iId, iIq;
	int iActualSpeed, iAngleFromHall, iUmot = 0; int iSpeedValueNew;
	int iPhase1Sum=0, iPhase2Sum=0, iPhase1=0, iPhase2=0, iCountRMS =0, iTriggerRMS=0, ia1RMSMax;	unsigned a1Square=0,a2Square=0;
	int iCountDivFactor, iVectorInvPark, cmd; unsigned iVectorCurrent; int iFieldSet = 0;
	int value_0[filter_length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, value_1[filter_length]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int value_d[filter_dc]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}, value_q[filter_dc]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	timer ts; int time, time1;
	int value_as[filter_dc]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int t1, t2, t3, t4 , t5;
	int e_d = 0, e_di = 0; int Kp_d = 0 , Ki_d = 0;
	int e_q = 0, e_qi = 0; int Kp_q = 0 , Ki_q = 0;
	int iId_f=0, iIq_f=0;
	char cTriggerPeriod=0; int iMotHoldingTorque = 0;
	int pf1=0, pf2=0;
	unsigned theta;  // angle
	int reached = 0;
	int Kd_q, e_qd, prev =0;
	int fil_cnt =0, fil_cnt1 =0;
	int ia_f=0, ib_f=0; int i =0;int fl=30; int j=0;
	int iq_fi=0, id_fi=0 , fdc = filter_dc, fph= filter_ph , fil_cnt_dc = 0; int flc =3; int mod, fldc = filter_dc ;
	int prev_v=0, dirn = 1;
	int am_new, ph1_new; int new_ia, new_ib;int th1, th2, am_fil; int flag =0;

	int max_count = QEI_COUNT_MAX_REAL / POLE_PAIRS;
	int zero_cross_count_min;

	int sync_position;
	int mod_speed;
	int zero_crossing = 0;
	int sign_p = 1, sign_c = 1;
	int cal_a1=0, cal_a2=0, cal_cnt = 0, cali_a1_cor=0;
	VsdRef1 = 0; VsqRef1 = 0;


	zero_cross_count_min = (max_count*4)/10;

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

	//	select{
	//		case ts when timerafter(time+5556) :> time:
				ts when timerafter(time+5556) :> time;
				{a1 , a2}  = get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);
				//xscope_probe_data(0, a1);
				//xscope_probe_data(1, a2);
			 	value_0[fil_cnt] = a1;
			 	value_1[fil_cnt] = a2;
				//break;
	//	}


//		ts when timerafter(time+300) :> void;

		sync_position = get_sync_position ( sync_output );

		xscope_probe_data(5, sync_position);


			/*c_hall_1<:5;
			c_hall_1:>dirn;
*/
			//xscope_probe_data(0, 0-a1);
			//xscope_probe_data(1, 0-a2);

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

		//post filter calib
      /*  cal_a1 += ia_f;
        cal_cnt++;
        if(cal_cnt > 1000)
        {
        	cali_a1_cor = cal_a1/cal_cnt;
        	cal_cnt = 0;cal_a1 = 0;
        }
        ia_f = ia_f - cali_a1_cor; */
		xscope_probe_data(0, a1);
		xscope_probe_data(1, a2);
		xscope_probe_data(2, ia_f);
				xscope_probe_data(3, ib_f);

		if(ia_f < 0)
		{
			sign_c = -1;
		}
		else if(ia_f >= 0)
		{
			sign_c = 1;
		}

		if(sign_c > sign_p)
		{

			zero_crossing = 1;
		}
		else if(sign_c < sign_p)
		{
			zero_crossing = -1;
		}
		else
		{
			zero_crossing = 0;
		}
		sign_p = sign_c;

			//	arctg1(int Real, int Imag)

			/*	zero_crossing = 0;
				if(ia_f ==0)
					zero_crossing = 1;*/
				xscope_probe_data(4, zero_crossing);



		iActualSpeed     = get_speed_cal(c_hall_1);;
			//iAngleFromHall   = 	get_hall_angle(c_hall_1) & 0x0FFF;


			mod_speed = iActualSpeed;
			if(iActualSpeed<0)
				mod_speed = 0 - iActualSpeed;
						if(mod_speed<370)
						{	flc  = 20;
						//	fldc = 80;
						}
						if(mod_speed < 100)
						{
							flc = 50;
						}
					/*	if(mod_speed>=370)
							{flc  = 30;
							fldc = 80;}
						if(mod_speed>=700)
							{flc  = 17;
							fldc = 80;}

						if(mod_speed>=1120 && mod_speed<=1378)
						{	flc  = 17;
							fldc = 80;
							//ia_f = -a1;
							//ib_f = -a2;
						}
						if(mod_speed>=1400)
						{	flc  = 7;
							fldc = 80;
							//ia_f = -a1;
							//ib_f = -a2;
						}
						if(mod_speed>=2000)
						{	flc  = 7;
							fldc = 80;
						}*/
						//ia_f=(0-ia_f);
					//	ib_f=(0-ib_f);
		/*	if(iActualSpeed > 0)
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

			//iPhase1Sum -= iPhase1;
			//iPhase1Sum += a1;
			iPhase1     =   ia_f;//iPhase1Sum/2;                     // is factor 2 for filtering the signal?

			//iPhase2Sum -= iPhase2;
			//iPhase2Sum += a2;
			iPhase2     = ib_f;//iPhase2Sum/2;						// is factor 2 for filtering the signal?

			iTemp = iPhase1;	 if(iTemp < 0) iTemp = -iTemp;a1Square += (iTemp * iTemp);iTemp = iPhase2;	 if(iTemp < 0) iTemp = -iTemp;a2Square += (iTemp * iTemp);
			iCountRMS++;

			//=================== RMS =====================
			if(iCountRMS > 18000) iTriggerRMS = 5; // 100.000  / 55,556  =  18000     timeout about 100msec

			if(cTriggerPeriod & 0x02)
			{
				 cTriggerPeriod &= 0x02^0xFF;
				 iTriggerRMS++;
			}

			if(iTriggerRMS > 1)
				if(iCountRMS)
				{
					a1SquareMean = a1Square/iCountRMS;
					a2SquareMean = a2Square/iCountRMS;
					a1Square  = 0;
					a2Square  = 0;
					iCountRMS = 0;
					if(iTriggerRMS >= 5) iTriggerRMS=0;
					else iTriggerRMS--;
				}


			if(iPhase1 > ia1RMSMax)   // save last max value
				ia1RMSMax = iPhase1;

			iAlpha =  iPhase1;
			iBeta  =  (iPhase1 + 2*iPhase2);   // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
			iBeta *=  37838;
			iBeta /=  65536;
			iBeta  =  -iBeta;

			 //============= park transform ==qq==========================
			theta = iAngleFromHall/16;			 // why is it divided by factor 16?			//xscope_probe_data(0,theta);
			th1 = (iAngleFromHall*100)/1138;
			//xscope_probe_data(1,th1);

			if(dirn==1)
			{
				th2 = th1 - 120;
				if(th2<0)
					th2=th2+360;
				th1= (120 + th1);
				if(th1>359)
					th1 =th1-359;
			}
			if(dirn == -1)
			{


				int cor = 0 , cor1 = 0;
				if(iActualSpeed<0 && iActualSpeed>-800)					//cor=10;
				{
					cor = 4900/iActualSpeed;  //70rpm 70deg
					cor1 = 2100/iActualSpeed;
					if(cor<-90)
						cor =-90;
					if(cor1<-90)
						cor1=-90;
					//cor1 = -90;
				}



				th2 = th1  -90;
				if(th2<0)
					th2=th2+359;
				th1 = th1 + 90 + cor ;   //th1+90 works for high speeds



				if(th1<0)
					th1= 359+th1;
				if(th1>359)
					th1 =  th1-359;



				//xscope_probe_data(0,th1);
			}




			theta &= 0xFF;
			sinx  = sine_table[theta];
			//xscope_probe_data(2,th1);

			ph1_new =(newsine_table[th1]);		//phase 1 angle
			theta = (64 - theta);  		    // 90-theta
			theta &= 0xFF;
			cosx  = sine_table[theta];      // values from 0 to +/- 16384

			iId = (((iAlpha * cosx )  /16384) + ((iBeta * sinx ) /16384));
			iIq = (((iBeta  * cosx )  /16384) - ((iAlpha * sinx ) /16384));


			am_new=prev_v;

			{
				if((th1>=246&&th1<=280) || (th1>=77&&th1<=105)  )
				{
					am_new= (iPhase1*16384)/ph1_new;
					prev_v =am_new;
					flag =0;
				}
				else
				{

					flag =1;
				}
			}


			if(flag !=1)
			{
				flag =0;
				value_as[fil_cnt1] = am_new;

				fil_cnt1 = (fil_cnt1+1)%fph ;

				am_fil = 0;
				j=0;
				for(i=0 ; i<40; i++)
				{
					mod = (fil_cnt1 - 1 - j)%fph;
					if(mod<0)
						mod = fph + mod;
					am_fil += value_as[mod];
					j++;
				}
				am_fil /= 40;
			}

			xscope_probe_data(0,iPhase1);
			xscope_probe_data(1,iPhase2);


				//xscope_probe_data(1,am_new);
				//xscope_probe_data(3,th1);
				//xscope_probe_data(2,iPhase1);
				//xscope_probe_data(4,iActualSpeed);
				//xscope_probe_data(0,am_fil);
				//xscope_probe_data(0,th2);
			     // phase1 is of different amp at + and - same powers
				//xscope_probe_data(1,a2);


			/*value_d[fil_cnt_dc] = iId;
			value_q[fil_cnt_dc] = iIq;
			fil_cnt_dc = (fil_cnt_dc+1)%fdc;
			id_fi  = 0; iq_fi = 0;
			j=0;
			for(i=0 ; i<20; i++)
			{
				mod = (fil_cnt_dc - 1 - j)%fdc;
				if(mod<0)
					mod = fdc + mod;
			//	xscope_probe_data(0, mod);

				id_fi += value_d[mod];
				iq_fi += value_q[mod];
				j++;
			}
			id_fi /= fldc;
			iq_fi /= fldc;

			//	ts :> t5;	xscope_probe_data(0, t5 - t4);
			//xscope_probe_data(0, th1);
			//xscope_probe_data(1, sign);
			/*xscope_probe_data(4, iId);
			xscope_probe_data(5, iIq);
			//xscope_probe_data(2, a1);
			//xscope_probe_data(3, a2);*/
				//xscope_probe_data(2, ia_f);
			//xscope_probe_data(3, ib_f);
			//xscope_probe_data(4,iActualSpeed);*/
	}
	//output is iUmotResult & iAngleInvPark

}



/*
 * 				{
						int sync_position = 0, time;
						timer t;
						t:>time;
						//t when timerafter(time+7*SEC_STD) :> time;

						while(1)
						{
							t when timerafter(time+1000) :> time;
							sync_position = get_sync_position ( sync_output );
						//	printintln(sync_position);
							xscope_probe_data(0, sync_position);

					//
						}

					}
 */

int main(void)
{
	chan c_adc, adc_d, dirn;
	chan c_adctrig;
	chan c_qei;
	chan c_hall, c_hall1;
	chan c_pwm_ctrl;
	chan c_commutation;
	chan speed_read;
	chan enco, dummy;
	chan pos_ctrl;
	chan pos_data;
	chan speed_out, stop, str, info, r_hall;
	chan enco_1, sync_output;
	chan signal_adc, c_value, input;

	//etherCat Comm channels
	chan coe_in; ///< CAN from module_ethercat to consumer
	chan coe_out; ///< CAN from consumer to module_ethercat
	chan eoe_in; ///< Ethernet from module_ethercat to consumer
	chan eoe_out; ///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; ///< File from module_ethercat to consumer
	chan foe_out; ///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;
	chan sig_1;
	//
	par
	{

		on stdcore[1]:
		{
			xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)",
					XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
					XSCOPE_DISCRETE, "3 ep", XSCOPE_INT, "n", XSCOPE_DISCRETE,
					"4 ev", XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "5 pos_d",
					XSCOPE_INT, "n", XSCOPE_CONTINUOUS, "6 vel_d", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "7 speed", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "8 sinepos_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "9 sinepos_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "10 sinepos_c", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "11 sine_a", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "12 sine_b", XSCOPE_UINT, "n",
					XSCOPE_CONTINUOUS, "13 sine_c", XSCOPE_UINT, "n");
			xscope_config_io(XSCOPE_IO_BASIC);
		}
		on stdcore[2]:{ //state for current


		//

			//synnn(sync_output);

			par{


				filter_loop(signal_adc, input, c_adc, r_hall, c_value, sync_output);

				hall_qei_sync(c_qei, c_hall1, sync_output);
			}
		}


		on stdcore[2]:{

		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]: {
			par {


				adc_ad7949_triggered( c_adc, c_adctrig, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

			//	adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port, p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_test(c_commutation, c_hall, c_pwm_ctrl, signal_adc);  // hall based

				//commutation_test(c_commutation, sync_output, c_pwm_ctrl, r_hall);  //new sync input based

				run_hall( c_hall, p_ifm_hall, c_hall1, r_hall);

				do_qei( c_qei, p_ifm_encoder );

				{

					unsigned time = 0;
					int speed;
					timer t;
					t:>time;
					t when timerafter(time+7*SEC_FAST) :> time;


					c_commutation <: 2;
					c_commutation <: 10500;

				}


			}
		}

	}

	return 0;
}


