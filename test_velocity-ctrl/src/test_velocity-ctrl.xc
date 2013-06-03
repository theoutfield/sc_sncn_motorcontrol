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
#include "hall_server.h"
#include "hall_client.h"
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "adc_ad7949.h"
#include "test.h"
#include "pwm_config.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include "adc_client_ad7949.h"
#include <dc_motor_config.h>
#include "sine_table_big.h"
#include "pos_ctrl.h"
#include "print.h"
#include "torque_ctrl.h"
#include "filter_blocks.h"

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;




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
	int index = 0, filter_output;
	int acc = 0; int old_fil = 0;

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
	ts when timerafter(time+1*SEC_FAST) :> time;

	while(1)
	{
		ts when timerafter(time+100000) :> time; //1khz
#ifdef test_sensor_qei_filter
		//{pos, v} = get_qei_position(c_qei);
		{pos, dirn} = get_qei_position_count(c_qei);
		diff = pos - prev;
		if(diff > 3080) diff = old;
		if(diff < -3080) diff = old;
		if(diff<0)
			diff = 0-diff;
		cal_speed = (diff*1000*60)/4000;

		filter_output = filter(filter_buffer, index, filter_length2, cal_speed);

		set_commutation_sinusoidal(c_torque, 3739);
	//	xscope_probe_data(0, pos);
	//	xscope_probe_data(1, cal_speed*dirn);
	//	xscope_probe_data(2, dirn);
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

int main(void) {
	chan c_adc, c_adctrig;
	chan c_qei;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_pwm_ctrl, c_commutation;
	chan dummy, dummy1, dummy2;
	chan speed_out, stop, str, info;
	chan enco_1, sync_output;
	chan signal_adc, c_value, input;
	chan c_torque;
	chan sig_1, signal_ctrl;
	chan c_position_ctrl;
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

	//
	par
	{
		on stdcore[0]:
		{
			//set_torque_test(c_torque);
			//torque_control( c_torque, c_hall_p4);
			// set_position_test(c_position_ctrl);

		/*	{//test

				ctrl_par velocity_ctrl_par;
				init_velocity_control(velocity_ctrl_par);
				printintln(velocity_ctrl_par.Kp_n);
				printintln(velocity_ctrl_par.Kp_d);
				printintln(velocity_ctrl_par.Ki_n);
				printintln(velocity_ctrl_par.Ki_d);
				printintln(velocity_ctrl_par.Kd_n);
				printintln(velocity_ctrl_par.Kd_d);
				printintln(velocity_ctrl_par.Loop_time);

				printintln(velocity_ctrl_par.Integral_limit);
				printintln(velocity_ctrl_par.Control_limit);

			}*/
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
			//par
			{
			//	speed_control(c_commutation, c_hall_p3, signal_ctrl, dummy);

			//	hall_qei_sync(c_qei, c_hall_p2, sync_output);


			}

			par{

				{
					ctrl_par velocity_ctrl_params;
					filt_par sensor_filter_params;
					hall_par hall_params;
					qei_par qei_params;

					init_velocity_control(velocity_ctrl_params);
					init_sensor_filter(sensor_filter_params);
					init_hall(hall_params);
					init_qei(qei_params);

					velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, qei_params, 2, c_hall_p2, c_qei, c_commutation);
				}

			}
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{

				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				commutation_sinusoidal(c_commutation, c_hall_p1, c_pwm_ctrl, signal_adc); 	// hall based sinus commutation

				//commutation_test(c_commutation, sync_output, c_pwm_ctrl, c_hall_p1);

				run_hall( p_ifm_hall, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4);  		// channel priority 1,2..4

				do_qei(c_qei, p_ifm_encoder);

			}
		}

	}

	return 0;
}
