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
#include "test.h"
#include "pwm_config.h"
#include "comm_sine.h"
#include "comm_loop.h"
#include "refclk.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include "sine_table_big.h"
#include "pos_ctrl.h"
#include "print.h"
#include "torque_ctrl.h"
#include "dc_motor.h"
#define COM_CORE 0
#define IFM_CORE 3
unsigned root_function(unsigned uSquareValue);
//on stdcore[3]: in           port    ADC_SYNC_PORT = XS1_PORT_16A;

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

#define FILTER_CURRENT_REQUEST 10
#define filter_length 30

{	int, int} get_filter_current(chanend c_filter_current) {
	int filterCurrentA, filterCurrentB;

c_filter_current	<: FILTER_CURRENT_REQUEST;
	c_filter_current :> filterCurrentA;
	c_filter_current :> filterCurrentB;

	return {filterCurrentA, filterCurrentB};
}
void filter_loop(chanend sig, chanend adc, chanend c_hall_1, chanend dummy,
		chanend c_filter_current) {
	int a1 = 0, a2 = 0;
	int iActualSpeed;
	int cmd;
	int value_0[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			value_1[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	timer ts;
	int time;
	int prev = 0;
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

			xscope_probe_data(0, ia_f);
			xscope_probe_data(1, ib_f);

			filter_count++;
			if(filter_count == 10)
			{
				filter_count = 0;
				iActualSpeed = get_speed_cal(c_hall_1);

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

			case c_filter_current :> cmd: // < 3-5 KHz ~ 0.2 ms
			if(cmd == FILTER_CURRENT_REQUEST)
			{
				c_filter_current <: ia_f; // send out only single phase
				c_filter_current <: ib_f; // send both phase
			}
			break;
		}

	}

}

#define filter_dc 160
#define defParAngleUser 560

void foc_torque_ctrl_loop(chanend sig, chanend adc, chanend c_hall_1,
		chanend sync_output, chanend c_filter_current, chanend c_value) {
	int a1 = 0, a2 = 0;
	int iActualSpeed;
	int cmd;
	int value_0[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
			value_1[filter_length] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
					0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	timer ts;
	int time;
	int prev = 0;
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

	//tor ctrl
	int iAnglePWM, iAnglePWMFromFOC, iAngleInvPark, i_hall_angle;
	int sinx, cosx, iTorqueSet = 130, iAlpha, iBeta;
	int VsdRef1, VsqRef1, VsdRef2, VsqRef2; // invers park
	int VsaRef, VsbRef, iId, iIq;
	int iUmot = 0;
	int iPhase1 = 0, iPhase2 = 0;
	int iVectorInvPark;
	unsigned iVectorCurrent;
	int iFieldSet = 0;
	int value_d[filter_dc], value_q[filter_dc];
	timer tc;
	int time1;
	int e_d = 0, e_di = 0;
	int e_q = 0, e_qi = 0;
	unsigned theta; // angle
	int i1 = 0, j1 = 0, mod1 = 0;
	int Kd_q, e_qd;

	int iq_fi = 0, id_fi = 0, fdc = filter_dc, fil_cnt_dc = 0;
	int fldc = filter_dc, Speed = 0;
	int prev_q = 0, prev_d = 0, e_dv = 0, e_qv = 0;
	int Kp1 = 40, Kv1 = 0, Ki1 = 4;
	int Kp2 = 40, Kv2 = 0, Ki2 = 4;
	int t_actual = 0;
	int wait = SEC_STD;

	int taninv;
	VsdRef1 = 0;
	VsqRef1 = 0;

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
	tc :> time1;

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
			//	xscope_probe_data(2, i_hall_angle);

			//ia_f = 0-ia_f;
			//ib_f = 0-ib_f;
			//{ia_f, ib_f} = get_filter_current(c_filter_current);
			//	xscope_probe_data(0, ia_f);
			//	xscope_probe_data(1, ib_f);

			iPhase1 = 0 - ia_f;
			iPhase2 = 0 - ib_f;

			iAlpha = iPhase1;
			iBeta = (iPhase1 + 2*iPhase2); // iBeta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732
			iBeta *= 37838;
			iBeta /= 65536;
			iBeta = -iBeta;

			//============= park transform ============================
			theta = i_hall_angle; //range 500 qei
			theta &= 499;
			sinx = newsine_table[theta]; // sine( theta );
			theta = (125 - theta); // 90-theta
			theta &= 499; //499
			cosx = newsine_table[theta]; // values from 0 to +/- 16384

			iId = (((iAlpha * cosx ) /16384) + ((iBeta * sinx ) /16384));
			iIq = (((iBeta * cosx ) /16384) - ((iAlpha * sinx ) /16384));

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

			t_actual = root_function(iq_fi*iq_fi+id_fi*id_fi);
			xscope_probe_data(0, t_actual);
			xscope_probe_data(1, iTorqueSet);

			xscope_probe_data(2, id_fi);
			xscope_probe_data(3, iq_fi); //torque

			Speed = get_speed_cal(c_hall_1);

			if(Speed<370)
			{
				fldc = 160;
			}
			else if( Speed > 370 && Speed <1000)
			{
				fldc = 70;
			}
			else
			{
				fldc = 25;
			}

			e_d = (iFieldSet - id_fi);
			e_dv = e_d - prev_d;

			//VsdRef2 = (f_param.Kp_n * e_d)/f_param.Kp_d + (e_di * f_param.Ki_n)/f_param.Ki_d;		//10
			//	Kp1= 20; Kv1 = 0; Ki1 = 4;

			Kp1= 27; Kv1 = 1; Ki1 =57;//2
			VsdRef2 = (Kp1 * e_d)/10 + (Ki1 * e_di)/1000 +(Kv1 * e_dv)/10;
			e_di += e_d;
			prev_d = e_d;

			//Kp_q = 40; Ki_q = 4; 								//Kd_q= 2; //20  12
			//	Kp2 =140; Kv2 = 2; Ki2 = 7;

			Kp2 =60; Kv2 = 0; Ki2 = 7;
			e_q = (iTorqueSet - iq_fi);
			e_qv = e_q - prev_q;
			//VsqRef2 = (t_param.Kp_n * e_q)/t_param.Kp_d + (e_qi * t_param.Ki_n )/t_param.Ki_d  ;  			//10
			VsqRef2 = (Kp2 * e_q)/10 + (Ki2 * e_qi)/120 + (Kv2 * e_qv)/10;

			//xscope_probe_data(4,  (Kp2 * e_q)/10);
			//xscope_probe_data(5, (Kp1 * e_d)/10 );
			//	xscope_probe_data(3, iq_fi);

			e_qi += e_q;
			prev_q = e_q;

			//e_qd = e_q - prev;							//prev = e_q;


			if(e_qi > 10000)
			e_qi = 10000;
			else if(e_qi < -10000)
			e_qi = 0 - 10000;

			if(e_di > 10000)
			e_di = 10000;
			else if(e_di < -10000)
			e_di = 0 - 10000;

			if(VsqRef2 > 5000)
			VsqRef2 = 5000;
			if(VsdRef2 > 5000)
			VsdRef2 = 5000;

			if(VsqRef2 < -5000)
			VsqRef2 = -5000;
			if(VsdRef2 < -5000)
			VsdRef2 = -5000;

			//================= invers park transformation =====================
			VsaRef = VsdRef2 * cosx/16384 - VsqRef2 * sinx/16384;
			VsbRef = VsdRef2 * sinx/16384 + VsqRef2 * cosx/16384;
			iAngleInvPark = arctg1(VsaRef,VsbRef); // from 0 - 4095


			iVectorInvPark = VsaRef * VsaRef + VsbRef * VsbRef;
			iVectorInvPark = root_function(iVectorInvPark);

			//iVectorCurrent = iAlpha * iAlpha + iBeta * iBeta;
			//iVectorCurrent = root_function(iVectorCurrent);


			iUmot = iVectorInvPark/2;
			if(iUmot > 4050)
			iUmot = 4050; //pwm limit


			iAnglePWMFromFOC = iAngleInvPark + (4096 - 1020) + defParAngleUser;
			iAnglePWM = iAnglePWMFromFOC & 0x0FFF;

			iAnglePWM &= 0x0FFF;
			c_value <: 40;
			c_value <: iUmot; //to PWM loop 18Khz
			c_value <: iAnglePWM;

			break;

		}

	}

}

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


void set_torque_test(chanend c_torque) {
	int torque;
	in_data d;
	while (1) {
		input_cmd(d);
		printintln(d.set_torque);

		c_torque <: 2;
		c_torque <: d.set_torque;

	}
}

void torque_control(chanend c_hall_p4)
{
	/*variable declaration*/
	int J;
	int B;
	int coef_prec = 16; //2^16
	int var_prec = 5;   //2^5

	coeff c;

	int speed = 0, acc = 0, prev_speed = 0;
	timer ts;
	unsigned int time;
	int joint_torque_actual;
	int	joint_torque_set = 8000;
	int error_joint_torque;
	int error_joint_torque_I = 0;
	int error_joint_torque_D = 0;
	int error_joint_torque_previous = 0;
	int joint_torque_control_out = 0;

	int Kp, Ki, Kd;
	int TORQUE_INTEGRAL_MAX;
	int TORQUE_OUTPUT_MAX = 13700; //PWM max

	TORQUE_INTEGRAL_MAX = TORQUE_OUTPUT_MAX/Ki;
	/*compute motor constants from c function call*/

	c.prec = coef_prec;
	coeffient_prec(c);
	J = c.J;
	B = c.B;
	printintln(B);	printintln(J);

	ts :> time;
	//while(1)
	{
		//ts when timerafter(time+33333) :> time; //3khz
		/* computed current torque */
		speed = (5000*201)/60;  //rpm to rad with prec  //get_speed_cal(c_hall_p4)
		acc = (speed - prev_speed)*333; 			// already in rad/s2 with prec

		joint_torque_actual = (J*acc + B*speed)>>var_prec;  // in coef_prec only

		prev_speed = acc;
		printintln(speed);	printintln(acc);
		printintln(joint_torque_actual);

		/* control torque set point*/
		joint_torque_set = (joint_torque_set << coef_prec)/1000; // in coef_prec only
		printintln(joint_torque_set);


		/* torque controller*/
		error_joint_torque = joint_torque_set - joint_torque_actual;
		error_joint_torque_I = error_joint_torque_I + error_joint_torque;
		error_joint_torque_D = error_joint_torque - error_joint_torque_previous;

		if(error_joint_torque_I > TORQUE_INTEGRAL_MAX)
		{
			error_joint_torque_I = TORQUE_INTEGRAL_MAX;
		}
		else if(error_joint_torque_I < 0)
		{
			error_joint_torque_I = 0 ;
		}

		joint_torque_control_out = Kp * error_joint_torque + Ki * error_joint_torque_I + Kd * error_joint_torque_D;

		if(joint_torque_control_out >= TORQUE_OUTPUT_MAX) {
			joint_torque_control_out = TORQUE_OUTPUT_MAX;
		}
		else if(joint_torque_control_out < 0){
			joint_torque_control_out = 0;
		}

		error_joint_torque_previous = error_joint_torque;
	}

}



int main(void) {
	chan c_adc, adc_d, dirn;
	chan c_adctrig;
	chan c_qei;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_pwm_ctrl;
	chan c_commutation;
	chan speed_read;
	chan enco, dummy, dummy1, dummy2;
	chan pos_ctrl;
	chan pos_data;
	chan speed_out, stop, str, info;
	chan enco_1, sync_output;
	chan signal_adc, c_value, input;

	chan c_filter_current;
	chan c_phase_position;

	chan phase_sync;

	chan c_torque;

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

		on stdcore[0]:
		{
			//set_torque_test(c_torque);

		}

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
		{
			par
			{
				//filter_loop(signal_adc, c_adc, r_hall, dummy2, c_filter_current);
				//foc_torque_ctrl_loop(signal_adc, c_adc, r_hall1, sync_output, c_filter_current, c_value);

			/*	current_ctrl_loop(signal_adc, c_adc, c_hall_p3,
						sync_output, c_filter_current, c_commutation, c_torque);

				hall_qei_sync(c_qei, c_hall_p2, sync_output);
*/

				torque_control(c_hall_p4);


				//torque_ctrl( c_value, c_filter_current, sync_output, r_hall1);
			}
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

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_test(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc); // hall based

				//commutation_test(c_commutation, sync_output, c_pwm_ctrl, r_hall);  //new sync input based

				//commutation(c_value, c_pwm_ctrl, signal_adc);	 //

				run_hall(c_hall_p1, p_ifm_hall, c_hall_p2, c_hall_p3, c_hall_p4);  // channel priority 1,2..4

				do_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}


