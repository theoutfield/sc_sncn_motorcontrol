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

#define PHASE_CORRECTION_REQUEST 30
#define SYNC_PHASE_POSITION_REQUEST 20
#define FILTER_CURRENT_REQUEST 10


#define filter_length 30
#define filter_dc 160
#define filter_ph 160



void set_phase_position ( chanend sync_output, int  phase_position)
{
	sync_output <: PHASE_CORRECTION_REQUEST;
	sync_output <: phase_position;

	return;
}

void filter_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1,
		chanend c_value, chanend sync_output, chanend c_filter_current) {
	int iAngleFromHallOld = 0, iAnglePWM, iAnglePWMFromHall, iAnglePWMFromFOC;
	int iTemp;
	int a1 = 0, a2 = 0;
	int sinx, cosx, iTorqueSet = 50, iAlpha, iBeta, a1SquareMean = 0,
			a2SquareMean = 0, a1RMS, a2RMS;
	int VsdRef1, VsqRef1, VsdRef2, VsqRef2; // invers park
	int VsaRef, VsbRef, iAngleInvPark, iId, iIq;
	int iActualSpeed, iAngleFromHall, iUmot = 0;
	int iSpeedValueNew;
	int iPhase1Sum = 0, iPhase2Sum = 0, iPhase1 = 0, iPhase2 = 0,
			iCountRMS = 0, iTriggerRMS = 0, ia1RMSMax;
	unsigned a1Square = 0, a2Square = 0;
	int iCountDivFactor, iVectorInvPark, cmd;
	unsigned iVectorCurrent;
	int iFieldSet = 0;
	int value_0[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			value_1[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int value_d[filter_dc] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, value_q[filter_dc] = { 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0 };
	timer ts;
	int time, time1;
	int value_as[filter_dc] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	int t1, t2, t3, t4, t5;
	int e_d = 0, e_di = 0;
	int Kp_d = 0, Ki_d = 0;
	int e_q = 0, e_qi = 0;
	int Kp_q = 0, Ki_q = 0;
	int iId_f = 0, iIq_f = 0;
	char cTriggerPeriod = 0;
	int iMotHoldingTorque = 0;
	int pf1 = 0, pf2 = 0;
	unsigned theta; // angle
	int reached = 0;
	int Kd_q, e_qd, prev = 0;
	int fil_cnt = 0, fil_cnt1 = 0;
	int ia_f = 0, ib_f = 0;
	int i = 0;
	int fl = 30;
	int j = 0;
	int iq_fi = 0, id_fi = 0, fdc = filter_dc, fph = filter_ph, fil_cnt_dc = 0;
	int flc = 3;
	int mod, fldc = filter_dc;
	int prev_v = 0, dirn = 1;
	int am_new, ph1_new;
	int new_ia, new_ib;
	int th1, th2, am_fil;
	int flag = 0;

	int mod_speed;

	int max_count = QEI_COUNT_MAX_REAL / POLE_PAIRS;
	int zero_cross_count_min;
	int sync_position;
	int zero_crossing = 0;
	int sign_p = 1, sign_c = 1;
	int z_count = 0, diffe = 0, u_cur = 0, previous = 0, first1 = 1;
	int phase_offset = 0;
	int phase_corrected_position = 0;

	int cal_a1 = 0, cal_a2 = 0, cal_cnt = 0, cali_a1_cor = 0;

	int filter_count = 0;
	VsdRef1 = 0;
	VsqRef1 = 0;

	zero_cross_count_min = (max_count * 40) / 100;  // atleast 80 percent of 180 deg


	while (1) {
		unsigned cmd, found = 0;
		select
		{
case			sig :> cmd:
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
	ts :> time;
	ts :> t1;

	while(1)
	{
#pragma ordered
		select
		{


			case ts when timerafter(time+5556) :> time:				// .05 ms

				{	a1 , a2 }= get_adc_vals_calibrated_int16_ad7949(adc); //get_adc_vals_raw_ad7949(adc);

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

				//post filter calib
				/*  cal_a1 += ia_f;
				 cal_cnt++;
				 if(cal_cnt > 1000)
				 {
				 cali_a1_cor = cal_a1/cal_cnt;
				 cal_cnt = 0;cal_a1 = 0;
				 }
				 ia_f = ia_f - cali_a1_cor; */
			//	xscope_probe_data(0, a1);
			//	xscope_probe_data(1, a2);
			//	xscope_probe_data(2, ia_f);



				//xscope_probe_data(5, sync_position);

				//xscope_probe_data(3, z_count);

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
					sync_position = get_sync_position ( sync_output );  //0
					// add it as phase correction
					set_phase_position ( sync_output, sync_position);

				//	phase_offset = sync_position;
					//z_count = 0;
					//previous = sync_position;
									//reset z_count after approval of crossing
				}
				else if(sign_c < sign_p)
				{
					//phase_offset = sync_position;
						zero_crossing = -1;
						//z_count = 0;
						//previous = sync_position;
										//reset z_count after approval of crossing
				}
				else
				{
					zero_crossing = 0;
				/*	sync_position = get_sync_position ( sync_output );
					phase_corrected_position = (sync_position +  phase_offset);
							if(phase_corrected_position < 0)
								phase_corrected_position = max_count + phase_corrected_position;
							if(phase_corrected_position > max_count)
								phase_corrected_position = phase_corrected_position - max_count;*/
				}
				sign_p = sign_c;



			//	xscope_probe_data(3, zero_crossing);
			//	sync_position = get_sync_position ( sync_output );
			//	xscope_probe_data(4, sync_position);


			/*	if((phase_corrected_position>=50&&phase_corrected_position<=200)  )
				{
					am_new= (ia_f*16384)/newsine_table[phase_corrected_position];
					prev_v =am_new;
					flag =0;
				}
				xscope_probe_data(6, am_new);
		*/

				filter_count++;
				if(filter_count == 10)
				{
					filter_count = 0;
					iActualSpeed = get_speed_cal(c_hall_1);;

					mod_speed = iActualSpeed;
					if(iActualSpeed<0)
					mod_speed = 0 - iActualSpeed;
					if(mod_speed<370)
					{	flc = 20;
						//	fldc = 80;
					}
					if(mod_speed < 100)
					{
						flc = 50;
					}
				}
				break;




			case c_filter_current :> cmd:						// < 3-5 KHz ~ 0.2 ms
				if(cmd == FILTER_CURRENT_REQUEST)
				{
					c_filter_current <: ia_f;  // send out only single phase
				}
				break;
		}

	}

}
int get_filter_current(chanend c_filter_current)
{
	int filterCurrent;

	c_filter_current <: FILTER_CURRENT_REQUEST;
	c_filter_current :> filterCurrent;

	return filterCurrent;
}

int get_phase_position ( chanend c_phase_position )
{
	int phasePosition;

	c_phase_position <: SYNC_PHASE_POSITION_REQUEST;
	c_phase_position :> phasePosition;

	return phasePosition;
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




int main(void) {
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

	chan c_filter_current;
	chan c_phase_position;

	chan phase_sync;
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
			xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "1 qei", XSCOPE_INT, "n",
					XSCOPE_CONTINUOUS, "2 pos", XSCOPE_INT, "n",
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

		on stdcore[2]:
		{ //state for current


			par {

				filter_loop(signal_adc, input, c_adc, r_hall, c_value,
						phase_sync, c_filter_current);

				//hall_qei_sync(c_qei, c_hall1, sync_output);
				{

					int cmd; // Command token

					int hall_position; // Hall input
					int qei_position; // Qei input
					int sync_position = 0; // output

					int qei_valid; // qei validity (0 or 1)

					int hall_max_position = 4095;

					timer t_qei;
					unsigned int time_qei;
					int init=1, diffi = 0;
					int phase_correct = 0;
					int previous_position = 0;

					int max_count = QEI_COUNT_MAX_REAL / POLE_PAIRS;

					while(1)
					{
						#pragma ordered
						select
						{
							case t_qei when timerafter(time_qei + 300) :> time_qei:
								{	qei_position, qei_valid}= get_qei_position( c_qei); //aquisition
								//if(qei_valid){
								qei_position = hall_max_position - qei_position;

								if(init == 1)
								{
									previous_position = qei_position;
									init=0;
								}
								if(previous_position!= qei_position )
								{
									diffi = qei_position - previous_position;
									if( diffi > 3000)
									{
										sync_position = sync_position - 1;
									}
									else if(diffi < -3000)
									{
										sync_position = sync_position + 1;
									}
									else if( diffi < 10 && diffi >0)
									{
										sync_position = sync_position + diffi;
									}
									else if( diffi < 0 && diffi > -10)
									{
										sync_position = sync_position - diffi;
										if(sync_position < 0)
										{
											sync_position = max_count + sync_position;
										}
									}
									previous_position=qei_position;
								}
								if(sync_position >= max_count)
								{
									sync_position = 0;
								}

								//
								//xscope_probe_data(4, sync_position);
								//}
								break;



							case phase_sync :> cmd:
								if(cmd == SYNC_PHASE_POSITION_REQUEST)  	//corrected position out
								{
									phase_sync <: sync_position;
								}
								else if ( cmd == PHASE_CORRECTION_REQUEST)	//correction in
								{
									phase_sync :> phase_correct;
									sync_position = 0;   //only correction on one side
									/*if(sync_position < 0)
										sync_position = max_count + sync_position;
									if(sync_position > max_count)
										sync_position = sync_position - max_count;*/
								}
								break;

							case c_phase_position :> cmd:
								if(cmd == SYNC_PHASE_POSITION_REQUEST)  	//corrected position out
								{
									c_phase_position <: sync_position;
								}
								break;

						}
					}
				}



				{
					int filterCurrent;
					int phasePosition;
					timer t;
					int ctrl_time;

					t :> ctrl_time;
					while(1)
					{
						t when timerafter(ctrl_time + 20000) :> ctrl_time;

						filterCurrent = get_filter_current(c_filter_current);
						xscope_probe_data(0, filterCurrent);
						phasePosition =  get_phase_position (c_phase_position);
						xscope_probe_data(1, phasePosition);


					}

				}


			}
		}

		on stdcore[1]:
		{
		/*	{
				while(1)
				{
					int filterCurrent;
					int phasePosition;
					filterCurrent = get_filter_current(c_filter_current);
					phasePosition = get_phase_position(c_phase_position);




				}
			}*/
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par {

				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				//	adc_ad7949( c_adc, clk_adc, p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa, p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_test(c_commutation, c_hall, c_pwm_ctrl, signal_adc); // hall based

				//commutation_test(c_commutation, sync_output, c_pwm_ctrl, r_hall);  //new sync input based

				run_hall(c_hall, p_ifm_hall, c_hall1, r_hall);

				do_qei(c_qei, p_ifm_encoder);

				{

					unsigned time = 0;
					int speed;
					timer t;
t					:>time;
					t when timerafter(time+7*SEC_FAST) :> time;

					c_commutation <: 2;
					c_commutation <: 5500;

				}

			}
		}

	}

	return 0;
}

