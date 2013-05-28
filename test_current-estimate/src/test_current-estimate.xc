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
#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3
unsigned root_function(unsigned uSquareValue);
//on stdcore[3]: in           port    ADC_SYNC_PORT = XS1_PORT_16A;

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

/* fixed length digital filter (moving average filter)*/
void init_filter(int filter_buffer[], int index[], int filter_length)
{
	int i;
	for(i=0; i<filter_length; i++)
	{
		filter_buffer[i] = 0;
	}
	index[0] = 0;
}
int filter(int filter_buffer[], int index[], int filter_length, int input)
{
	int i, j = 0, mod, filter_output =0;
	filter_buffer[index[0]] = input;
	index[0] = (index[0]+1)%(filter_length);

	for(i=0; i<filter_length; i++)
	{
		mod = (index[0] - 1 - j)%filter_length;
		if(mod<0)
			mod = filter_length + mod;
		filter_output += filter_buffer[mod];
		j++;
	}
	filter_output = filter_output/ filter_length;
	return filter_output;
}

int get_torque(chanend c_torque)
{
	int torque;
	c_torque <: 3;
	c_torque :> torque;
	return torque;
}
void set_commutation(chanend c_commutation, int input)
{
	c_commutation <: 2;
	c_commutation <: input;
	return;
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

void set_position_test(chanend c_position_ctrl) {
	int position = 0;
	in_data d;
	timer ts;
	unsigned time;
	int increment = 50;

	ts:>time;


	while (1) {
		//input_pos(d);
		//printintln(d.set_position);

		ts when timerafter(time+100000) :> time;

		position +=increment;
		c_position_ctrl <: 2;
		c_position_ctrl <: position;
		if(position>300000)
			increment *= -1;
		if(position<0)
			increment *= -1;

	}
}
//printintln(speed);	printintln(acc); printintln(joint_torque_actual); printintln(B);	printintln(J); printintln(joint_torque_set);

void torque_control(chanend c_torque, chanend c_hall_p4)
{
	/*variable declaration*/
	int J;
	int B;
	int coef_prec = 15; //2^16
	int var_prec = 5;   //2^5

	coeff c;
	pd_data p;
	int speed = 0, acc = 0, prev_speed = 0;
	timer ts;
	unsigned int time, time1;
	int joint_torque_actual;
	int	joint_torque_set = 13;
	int error_joint_torque;
	int error_joint_torque_I = 0;
	int error_joint_torque_D = 0;
	int error_joint_torque_previous = 0;
	int joint_torque_control_out = 0;

	int Kp, Ki, Kd;
	int TORQUE_INTEGRAL_MAX;
	int TORQUE_OUTPUT_MAX = 13700; //PWM max
int sp1;


	#define filter_length1 1
	int filter_buffer[filter_length1];
	int index[] = {0}, filter_output;

	init_filter(filter_buffer, index, filter_length1);



//	TORQUE_INTEGRAL_MAX = TORQUE_OUTPUT_MAX/Ki;
	/*compute motor constants from c function call*/

	c.coef_prec = coef_prec;
	c.var_prec = var_prec;

	init_constants(c);


	/*ts :> time;
	p.speed = 5000;
	p.prev_speed = 5200;
	p.joint_torque_set = 13700;
	p.previous_error = 5200;
	pd_control(p);
	ts:> time1;
	printintln(time1-time); //33khz comput time*/

	ts :> time;
	 //set_torque(c_torque, 10000);
	while(1)
	{set_torque(c_torque, 10000);
		ts when timerafter(time+33333) :> time; //3khz

		/* computed current torque */
		//c_torque<:4;
		//c_torque:>speed;
		speed = get_speed_cal(c_hall_p4);

#ifdef ENABLE_xscope_main
		//xscope_probe_data(0,speed);
		xscope_probe_data(0, acc/1024);
		xscope_probe_data(1, speed);
		//xscope_probe_data(2,joint_torque_actual);
#endif

		acc = ((((speed - prev_speed)*3000)/60)*201)/32;
		//filter_output = filter(filter_buffer, index, filter_length1, acc);
		sp1= (speed*201)/(60*32);
		joint_torque_actual = acc+speed;
	/*	p.speed = speed;
		p.prev_speed = prev_speed;

		// control torque set point
		joint_torque_set = 1300; 		// in coef_prec only
		p.joint_torque_set = joint_torque_set;
		p.previous_error = error_joint_torque_previous;

		// torque controller call c function
		pd_control(p);

		joint_torque_control_out = p.joint_torque_control_out;

		if(joint_torque_control_out >= TORQUE_OUTPUT_MAX) {
			joint_torque_control_out = TORQUE_OUTPUT_MAX;
		}
		else if(joint_torque_control_out < 0){
			joint_torque_control_out = 0;
		}
		set_torque(c_torque, 10000);
		//set_torque(c_torque, joint_torque_control_out);
*/
#ifdef ENABLE_xscope_main
		//xscope_probe_data(0,p.joint_torque_actual);
#endif
	//	error_joint_torque_previous = p.error;
		prev_speed = speed;
	}

}


void position_control(chanend c_torque, chanend c_hall_p4, chanend c_position_ctrl)
{
	int actual_position = 0;
	timer ts;
	unsigned int time;
	int error_position = 0;
	int error_position_D = 0;
	int error_position_I = 0;
	int previous_error = 0;
	int position_control_out = 0;
	int Kp = 6, Kd = 0, Ki = 0;
	int max_integral = (13739)/1;
	int target_position = 15000;
	int in_cmd;
	//get_hall_absolute_pos(chanend c_hall)

	ts:> time;
	ts when timerafter(time+3*SEC_FAST) :> time;
	//set_commutation(c_torque, 1000);
	while(1)
	{
		select{
			case c_position_ctrl :> in_cmd:
				c_position_ctrl :> target_position;
				break;
			default:
				break;
		}

		ts when timerafter(time+100000) :> time; //1khz

		actual_position = get_hall_absolute_pos(c_hall_p4);

		//xscope_probe_data(0, actual_position);

		error_position = (target_position - actual_position)*1000;
		error_position_I = error_position_I + error_position;
		error_position_D = error_position - previous_error;

		if(error_position_I > max_integral*1000)
			error_position_I = max_integral*1000;
		else if(error_position_I < -max_integral*1000)
			error_position_I = 0 - max_integral*1000;

		position_control_out = (Kp*error_position)/10000 + (Ki*error_position_I) + (Kd*error_position_D);

		if(position_control_out > 13739)
			position_control_out = 13739;
		else if(position_control_out < -13739)
			position_control_out = 0-13739;

		//set_torque(c_torque, speed_control_out);
		set_commutation(c_torque, position_control_out);

		#ifdef ENABLE_xscope_main
		xscope_probe_data(0, actual_position);
		xscope_probe_data(1, target_position);
		#endif

		previous_error = error_position;

	}
}


int get_enc_pos(chanend enco)
{
	int pos;
	enco <: 1;
	enco :> pos;
	return pos;
}

{int, int} get_pos_count(chanend c_qei)
{
	int pos, dirn; // clkwise +ve  cclkwise -ve
	c_qei <: 2;
	master {
		c_qei :> pos;
		c_qei :> dirn;
	}
	//if(pos < 0)
	//	pos = 0 - pos;  not good idea works for only speed
	//pos = dirn*pos;
	return {pos, dirn};
}
//#define test_sensor_filter
//#define speedcontrol
#define test_sensor_qei_filter

void speed_control(chanend c_torque, chanend c_hall_p4, chanend signal2, chanend c_qei)
{
	int actual_speed = 0;
	timer ts;
	unsigned int time;
	int error_speed = 0;
	int error_speed_D = 0;
	int error_speed_I = 0;
	int previous_error = 0;
	int speed_control_out = 0;
	int Kp = 25, Kd = 1, Ki = 15;
 int max_integral = (13739*100)/Ki;
	int target_speed = 700;





#define filter_length2 8
	int filter_buffer[filter_length2];
	int index[] = {0}, filter_output;

int acc = 0; int old_fil = 0;

 /* while(1)
  {
	  unsigned cmd, found =0;
	  select
	  {
		case signal2 :> cmd:
			found = 1;
			break;
		default:
			break;
	  }
	  if(found == 1)
		  break;
  }*/
#ifdef test_sensor_filter
	int cal_speed, pos , prev =0, diff = 0, old = 0;
	int init=0;
#endif
#ifdef test_sensor_qei_filter
	int cal_speed, pos , prev =0, diff = 0, old = 0;
	int init=0; int v = 0;
	int dirn;
	//length 8
#endif

	init_filter(filter_buffer, index, filter_length2);

	ts:> time;
	ts when timerafter(time+3*SEC_FAST) :> time;

	while(1)
	{
		ts when timerafter(time+100000) :> time; //1khz
#ifdef test_sensor_qei_filter
		//{pos, v} = get_qei_position(c_qei);
		{pos, dirn} = get_pos_count(c_qei);
		diff = pos - prev;
		if(diff > 3080) diff = old;
		if(diff < -3080) diff = old;
		if(diff<0)
			diff = 0-diff;
		cal_speed = (diff*1000*60)/4000;

		filter_output = filter(filter_buffer, index, filter_length2, cal_speed);

		set_commutation(c_torque, 13090);
		xscope_probe_data(0, pos);
		xscope_probe_data(1, cal_speed*dirn);
		xscope_probe_data(2, dirn);
		prev = pos;
		old = diff;
#endif
#ifdef test_sensor_filter
		pos = get_hall_absolute_pos(c_hall_p4);
		if(init==0)
		{
			if(pos>2049)
			{
				init=1;
				prev = 2049;
			}
			if(pos < -2049)
			{
				init=1;
				prev = -2049;
			}
			cal_speed = 0;
			//xscope_probe_data(0, (cal_speed*60)/(8*4095));
		}
		if(init==1)
		{
			diff = pos - prev;
			if(diff > 50000) diff = old;
			if(diff < -50000) diff = old;
			cal_speed = ((diff)*1000*60)/(8*4095);
			//if(cal_speed == 0) init=0;
			/*xscope_probe_data(0, cal_speed);
			actual_speed = get_speed_cal(c_hall_p4);
			xscope_probe_data(1, actual_speed);
			xscope_probe_data(2, pos);*/
			prev = pos;
			old =diff;
		}

		filter_output = filter(filter_buffer, index, filter_length2, cal_speed);

		acc = filter_output - old_fil;
		old_fil= filter_output;
		xscope_probe_data(0, filter_output);
		xscope_probe_data(1, acc*1000);

		//set_commutation(c_torque, 1339);


		error_speed = (target_speed - cal_speed)*1000;

		error_speed_I = error_speed_I + error_speed;
		error_speed_D = error_speed - previous_error;

		if(error_speed_I>max_integral*1000)
			error_speed_I = max_integral*1000;

		speed_control_out = (Kp*error_speed)/10000 + (Ki*error_speed_I)/100000 + (Kd*error_speed_D)/1000;

		if(speed_control_out > 13739)
			speed_control_out = 13739;

		//set_torque(c_torque, speed_control_out);
		set_commutation(c_torque, speed_control_out);

		#ifdef ENABLE_xscope_main
		//xscope_probe_data(0, cal_speed);
		//xscope_probe_data(1, target_speed);
		#endif

		previous_error = error_speed;
#endif
#ifdef speedcontrol
		actual_speed = get_speed_cal(c_hall_p4);

		error_speed = (target_speed - actual_speed)*1000;

		error_speed_I = error_speed_I + error_speed;
		error_speed_D = error_speed - previous_error;

		if(error_speed_I>max_integral*1000)
			error_speed_I = max_integral*1000;

		speed_control_out = (Kp*error_speed)/10000 + (Ki*error_speed_I)/100000 + (Kd*error_speed_D)/1000;

		if(speed_control_out > 13739)
			speed_control_out = 13739;

		//set_torque(c_torque, speed_control_out);
		set_commutation(c_torque, speed_control_out);

		#ifdef ENABLE_xscope_main
		xscope_probe_data(0, actual_speed);
		xscope_probe_data(1, target_speed);
		#endif

		previous_error = error_speed;
#endif

	}

}
void encoder(chanend enco, chanend c_qei, qei_par &q_max)
{
	  unsigned pos_cmd;
	  unsigned uvalue[3];
	  signed pos, spos, prev=0, count=0, co12=0,first=1,set=0;
	  int max_count = 26 * 4000;
	  int hall_p;
	  int diffi, count1;
	  unsigned time = 0;
	  timer t; t:>time;
	  init_qei(q_max);
	  while(1)
	  {
			// data acquisition loop
			select
			{
				case enco:>pos_cmd:
					if(pos_cmd==1)
					{
						enco <: count;
					}
				 break;

				default:
				 break;
			} // data send loop


			//start encoder acquisition , qei_par &q_max)
			{uvalue[0], uvalue[1], uvalue[2]} = get_qei_data(c_qei, q_max);
			 t when timerafter(time+6250) :> time;
			if(first==1 )
			{
				if (uvalue[2]==1)
				{
					prev=uvalue[1]; first=0; set=prev;
				}
			}

			if(prev!= uvalue[1] && uvalue[2]==1 )
			{
				spos=uvalue[1];

				diffi = spos-prev;
				  if( diffi > 3000)
				  {
				      count = count - 1;
				  }
				  else if(diffi < -3000)
				  {
					  count = count + 1;
				  }
				  else if( diffi < 10 && diffi >0)
				  {
					  count = count + diffi;
				  }
				  else if( diffi < 0 && diffi > -10)
				  {
					  count = count + diffi;
				  }

				prev=spos;
				if(prev==set)
					co12++;
			}


			if(count >= max_count || count <= -max_count)
			{
				co12=0;count=0;

			}


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
	chan sig_1, signal_ctrl;
	chan c_position_ctrl;
	//
	par
	{

		on stdcore[0]:
		{
			//set_torque_test(c_torque);
			//torque_control( c_torque, c_hall_p4);
			// set_position_test(c_position_ctrl);

		}

		on stdcore[1]:
		{
			xscope_register(14, XSCOPE_CONTINUOUS, "0 hall(delta)", XSCOPE_INT,
					"n", XSCOPE_CONTINUOUS, "1 actualspeed", XSCOPE_INT, "n",
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
                /*in order of priority*/

			/*	hall_qei_sync(c_qei, c_hall_p2, sync_output);

				current_ctrl_loop(signal_adc, signal_ctrl, c_adc, c_hall_p3,
						sync_output, c_commutation, c_torque);
			 */
				//
				speed_control(c_commutation, c_hall_p4, signal_ctrl, c_qei);


			//	position_control(c_commutation, c_hall_p2, c_position_ctrl);

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

				//torque_control(c_hall_p4);

				do_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}

/*
 *#define FILTER_CURRENT_REQUEST 10
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
		//	xscope_probe_data(0, t_actual);
		//	xscope_probe_data(1, iTorqueSet);

		//	xscope_probe_data(2, id_fi);
		//	xscope_probe_data(3, iq_fi); //torque

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
 *
 */
