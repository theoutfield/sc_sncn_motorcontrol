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
#include "hall_server.h"
#include "hall_client.h"
#include "qei_client.h"
#include "pwm_service_inv.h"
#include "comm_loop.h"
#include "refclk.h"
#include "velocity_ctrl.h"
#include <xscope.h>
#include "qei_client.h"
#include "qei_server.h"
#include <dc_motor_config.h>
#include "profile.h"
#include <flash_somanet.h>
#include <internal_config.h>

#include <drive_config.h>

#define ENABLE_xscope_main
#define COM_CORE 0
#define IFM_CORE 3

on stdcore[IFM_CORE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_CORE]: clock clk_pwm = XS1_CLKBLK_REF;

void xscope_initialise_1()
{
	{
		xscope_register(2, XSCOPE_CONTINUOUS, "0 actual_velocity", XSCOPE_INT,	"n",
							XSCOPE_CONTINUOUS, "1 target_velocity", XSCOPE_INT, "n");

		xscope_config_io(XSCOPE_IO_BASIC);
	}
	return;
}

void forw_calib(commutation_par &commutation_params, int &angle_clk)
{
	int actual_velocity;
	angle_clk = angle_clk + 11;
	if(angle_clk > 1251)
		angle_clk = 1251;
}

//test PVM
void profile_velocity_test(chanend c_signal, chanend c_velocity_ctrl, chanend c_commutation, chanend c_hall, chanend c_qei)
{
	int i;
	int core_id = 1;

	int steps;
	int velocity_ramp;

	int actual_velocity = 0;     		// rpm
	int target_velocity =5000;	 		// rpm
	int acceleration = 2000;			// rpm/s      variable parameters
	int deceleration = 1050;			// rpm/s

	timer t;
	unsigned int time;

	int angle_clk = 45;
	int min_angle = 1*4095/360;
	int max_velocity_r = 0;

	pv_par pv_params;
	commutation_par commutation_params;
	hall_par hall_params;
	qei_par qei_params;


	int init = 0;
	int init_state = INIT_BUSY;

	init_pv_params(pv_params);
	init_commutation_param(commutation_params);
	init_hall_param(hall_params);
	init_qei_param(qei_params);

	acceleration = pv_params.profile_acceleration;   // or fixed parameters
	deceleration = pv_params.profile_deceleration;

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif
	while(1)
	{
		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
		//	printstrln("commutation intialized");
			break;
		}
	}
	if(init_state == INIT)
	{
		init_state = INIT_BUSY;
		init_state = init_velocity_control(c_velocity_ctrl);
		//if(init_state == INIT)	printstrln("velocity control intialized");
		//else			printstrln("intialize velocity control failed");
	}

	if(init_state == INIT)
	{
		steps = init_velocity_profile(target_velocity, actual_velocity, acceleration, deceleration, 8000);

		for(i = 1; i < steps; i++)
		{
			wait_ms(1, core_id, t);
			velocity_ramp = velocity_profile_generate(i);
			set_velocity(velocity_ramp, c_velocity_ctrl);
			actual_velocity = get_velocity(c_velocity_ctrl);

			xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1,  get_qei_velocity( c_qei, qei_params));
			//xscope_probe_data(1, _get_hall_velocity_pwm_resolution(c_hall, hall_params));
		}

		while(1)
		{
			wait_ms(1, core_id, t);
			//commutation_params.angle_offset_clkwise = 600;
//			angle_clk = angle_clk + 1;
//			if(angle_clk > 1251)
//				angle_clk = 1251;
//			commutation_params.angle_offset_cclkwise = angle_clk;
//			set_commutation_params( c_commutation, commutation_params);
//			wait_ms(50, core_id, t);
//			actual_velocity = get_velocity(c_velocity_ctrl);

//			forw_calib(commutation_params, angle_clk);
//			commutation_params.angle_offset_clkwise = angle_clk;
//			set_commutation_params(c_commutation, commutation_params);
//			wait_ms(150, 1, t);
			actual_velocity = get_velocity(c_velocity_ctrl);
		//	printintln(actual_velocity);
			xscope_probe_data(0, actual_velocity);
			xscope_probe_data(1,  get_qei_velocity( c_qei, qei_params));
		//	xscope_probe_data(1, _get_hall_velocity_pwm_resolution(c_hall, hall_params));
		//	xscope_probe_data(1, angle_clk);
//			if(actual_velocity > max_velocity_r)
//				max_velocity_r = actual_velocity;
		//	printintln(max_velocity_r);


		}
	}
}


int get_avg_velocity(int sensor_select, chanend c_hall, hall_par &hall_params, int core_id, timer t, int &avg_times, chanend c_qei, qei_par &qei_params)
{
	int k;
	int velocity = 0;
	int actual_velocity;
	printstr(" sens ");printintln(sensor_select);printintln(avg_times);
	for(k = 0;k < avg_times ; k++)
	{
		if(sensor_select == 1)
		{
			actual_velocity = _get_hall_velocity_pwm_resolution(c_hall, hall_params);
			velocity = velocity + actual_velocity;
		}
		else if(sensor_select == 2)
		{
			actual_velocity = get_qei_velocity( c_qei, qei_params);
			velocity = velocity + actual_velocity;
		}
		wait_ms(1, core_id, t);
	}
	velocity = velocity/avg_times;
	return velocity;
}
void ramp_up(int &i, int comm_voltage, timer t, int core_id, chanend c_commutation)
{
	while( i < comm_voltage)
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + 10);
		wait_ms(5, core_id, t);
	}
}
void ramp_down(int &i, int comm_voltage, timer t, int core_id, chanend c_commutation)
{
	while(i > comm_voltage)
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i - 10);
		wait_ms(5, core_id, t);
	}
}
{int,int} update_comm_sine_max_state(int &sensor_select, timer t, int core_id, hall_par  &hall_params, int &avg_times, int max, chanend c_hall, chanend c_qei, qei_par qei_params)
{
	int s, actual_velocity;
	int samples = avg_times;
	printintln(samples);printstr(" sens ");printintln(sensor_select);
	wait_ms(150, core_id, t);
	actual_velocity = get_avg_velocity(sensor_select, c_hall, hall_params, core_id, t, samples, c_qei, qei_params);
	if(actual_velocity >= max)
		s = 1;
	else
		s = 0;
	return {s, actual_velocity};
}

void commutation_sine_automate(int &sensor_select, chanend c_signal, chanend c_commutation, commutation_par &commutation_params,\
		hall_par &hall_params, qei_par qei_params, chanend c_hall, chanend c_qei)
{
	timer t;
	int core_id = 1;
	unsigned int time;
	int input_voltage;
	int comm_voltage;  //3 stage 300 1500 max
	int max_nominal_speed = MAX_NOMINAL_SPEED;
	int max_reach_expected = (MAX_NOMINAL_SPEED * 85 )/100;
	int max_reached_speed_pos;
	int comm_max = 13739;
	int init_state;
	int i;
	int actual_velocity;
	int s1, s2, s3;
	int s4, s5, s6;
	int avg_times = 100;
	int k = 0;
	int sense = 0;
	int ok_positive = 0;
	int ok_negative = 0;
	int avg_speed_reach = 0;

	int pos_ok_f = 0;
	int neg_ok_f = 0;
	xscope_initialise_1();
	while(1)
	{

		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
			printstrln("commutation intialized");
			break;
		}
	}

	comm_voltage = (comm_max * 75 )/1000;
	i = 0;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);

	{s1,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, avg_times, (max_nominal_speed*75)/1000, c_hall, c_qei, qei_params);

	printintln(s1);
	//printintln(actual_velocity);

	comm_voltage = (comm_max * 375 )/1000;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);

	{s2,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, avg_times, (max_nominal_speed*375)/1000, c_hall, c_qei, qei_params);
	//printintln(actual_velocity);
	printintln(s2);

	comm_voltage = comm_max ;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);

	{s3,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, avg_times, max_reach_expected, c_hall, c_qei, qei_params);
	//printintln(actual_velocity);
	printintln(s3);

	max_reached_speed_pos = actual_velocity;

	printintln(max_reached_speed_pos);
	if(i<0)
		sense = 1;
	else if(i>0)
		sense = -1;
	while( i != 0 )
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + sense * 10);
		wait_ms(5, core_id, t);
	}

	if(s3 == 1)
		ok_positive = 1;

	commutation_params.max_speed_reached = max_reached_speed_pos;
	set_commutation_params( c_commutation, commutation_params);


	comm_voltage = -(comm_max * 75 )/1000;
	i = 0;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_avg_velocity(sensor_select, c_hall, hall_params, core_id, t, avg_times, c_qei, qei_params);
	//printintln(actual_velocity);
	if(actual_velocity <= - (max_nominal_speed*75)/1000)
		s4 = 1;
	else
		s4 = 0;
	printintln(s4);

	comm_voltage = -(comm_max * 375 )/1000;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_avg_velocity(sensor_select, c_hall, hall_params, core_id, t, avg_times, c_qei, qei_params);
	//printintln(actual_velocity);
	if(actual_velocity <= - (max_nominal_speed*375)/1000)
		s5 = 1;
	else
		s5 = 0;
	printintln(s5);

	comm_voltage = - comm_max;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_avg_velocity(sensor_select, c_hall, hall_params, core_id, t, avg_times, c_qei, qei_params);
	//printintln(actual_velocity);
	if(actual_velocity <= - max_reach_expected)
		s6 = 1;
	else
		s6 = 0;
	printintln(s6);
	//printintln(actual_velocity);

	if(i<0)
		sense = 1;
	else if(i>0)
		sense = -1;
	while( i != 0 )
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + sense * 10);
		wait_ms(5, core_id, t);
	}

	if(s6 == 1)
		ok_negative = 1;

	avg_speed_reach =  (0-actual_velocity + max_reached_speed_pos)/2;
	//printintln(avg_speed_reach);

	commutation_params.max_speed_reached = avg_speed_reach;
	set_commutation_params( c_commutation, commutation_params);

	printstrln("test ended");
	//printintln(ok_positive);printstr(" ");printintln(ok_negative);

	i = 0;
	comm_voltage = comm_max ;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);
	wait_ms(150, core_id, t);
	{s3,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, avg_times, max_reach_expected, c_hall, c_qei, qei_params);
	max_reached_speed_pos = actual_velocity;
	printintln(max_reached_speed_pos);
	if(i<0)
		sense = 1;
	else if(i>0)
		sense = -1;
	while( i != 0 )
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + sense * 10);
		wait_ms(5, core_id, t);
	}

	pos_ok_f = 0;
	if(actual_velocity >= max_reach_expected)
		pos_ok_f = 1;

	comm_voltage = - comm_max;
	i = 0;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);
	wait_ms(150, core_id, t);
	actual_velocity = get_avg_velocity(sensor_select, c_hall, hall_params, core_id, t, avg_times, c_qei, qei_params);
	printintln(actual_velocity);

	neg_ok_f = 0;
	if(actual_velocity <= - max_reach_expected)
		neg_ok_f = 1;


	if(neg_ok_f ==  1 && pos_ok_f == 1)
		printstrln("calibrated");
	if(i<0)
		sense = 1;
	else if(i>0)
		sense = -1;
	while( i != 0 )
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + sense * 10);
		wait_ms(5, core_id, t);
	}

}


int main(void) {
	chan c_adctrig;
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5 ;
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4;
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3;
	chan c_pwm_ctrl;
	chan c_signal_adc;
	chan c_sig_1, c_signal;
	chan c_velocity_ctrl;

	//etherCat Comm channels
	chan coe_in; 	///< CAN from module_ethercat to consumer
	chan coe_out; 	///< CAN from consumer to module_ethercat
	chan eoe_in; 	///< Ethernet from module_ethercat to consumer
	chan eoe_out; 	///< Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 	///< File from module_ethercat to consumer
	chan foe_out; 	///< File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;

	//
	par
	{
		on stdcore[0] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
					foe_in, pdo_out, pdo_in);
		}

		on stdcore[0] :
		{
			//firmware_update(foe_out, foe_in, c_sig_1); // firmware update
		}

		on stdcore[1]:
		{
	//		profile_velocity_test(c_signal, c_velocity_ctrl, c_commutation_p1, c_hall_p3, c_qei_p3);			// test PVM on slave side
			{
				int sensor_select = 2;
				commutation_par commutation_params;
				hall_par hall_params;
				qei_par qei_params;
				init_hall_param(hall_params);
				init_qei_param(qei_params);
				init_commutation_param(commutation_params); // initialize commutation params
				commutation_sine_automate(sensor_select, c_signal,  c_commutation_p1, commutation_params, hall_params, qei_params, c_hall_p3, c_qei_p3);
			}
		}

		on stdcore[2]:
		{
			par
			{

//				{
//					 ctrl_par velocity_ctrl_params;
//					 filt_par sensor_filter_params;
//					 hall_par hall_params;
//					 qei_par qei_params;
//
//					 init_velocity_control_param(velocity_ctrl_params);
//					 init_sensor_filter_param(sensor_filter_params);
//					 init_hall_param(hall_params);
//					 init_qei_param(qei_params);
//
//					 velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,\
//							 qei_params, 2, c_hall_p2, c_qei_p1, c_velocity_ctrl, c_commutation_p2);
//
//
//
////						 	int pos, v;
////						 	timer ts;
////						 	unsigned time;
////						 	qei_par qei_params;
////						 	 init_qei_param(qei_params);
////						 	ts:>time;
////						 	xscope_initialise_1();
////						 	while(1)
////						 	{
////						 		ts when timerafter(time+1000) :> time;
////						 		{pos, v} = get_qei_position(c_qei_p1, qei_params );
////						 		//printintln(pos);
////						 		xscope_probe_data(0, pos);
////						 		xscope_probe_data(1, v);
////						 	}
//
//				}

			}
		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on stdcore[IFM_CORE]:
		{
			par
			{

				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);


				{
					hall_par hall_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_commutation_param(commutation_params); // initialize commutation params
					commutation_sinusoidal(hall_params, commutation_params, c_hall_p1, c_pwm_ctrl, c_signal_adc, c_signal,
							c_commutation_p1, c_commutation_p2, c_commutation_p3);					 // hall based sinusoidal commutation
				}

				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(p_ifm_hall, hall_params, c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4); // channel priority 1,2..4
				}

				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(p_ifm_encoder, qei_params, c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4);  // channel priority 1,2..4
				}

			}
		}

	}

	return 0;
}
