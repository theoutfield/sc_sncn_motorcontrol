/*
 * auto_calib_hall.xc
 *
 *  Created on: Oct 4, 2013
 *      Author: pkanajar
 */
#include <auto_calib_hall.h>
#include <print.h>

int get_average_velocity(int sensor_select, chanend c_hall, hall_par &hall_params, qei_velocity_par &qei_velocity_params, int core_id, timer t, int &avg_times, chanend c_qei, qei_par &qei_params)
{
	int k;
	int velocity = 0;
	int actual_velocity;
	//printstr(" sens ");printintln(sensor_select);printintln(avg_times);
	for(k = 0;k < avg_times ; k++)
	{
		if(sensor_select == 1)
		{
			actual_velocity = get_hall_velocity(c_hall, hall_params);
			velocity = velocity + actual_velocity;
		}
		else if(sensor_select == 2)
		{
			actual_velocity = get_qei_velocity(c_qei, qei_params, qei_velocity_params);//get_qei_velocity( c_qei, qei_params);
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
{int, int} update_comm_sine_max_state(int &sensor_select, timer t, int core_id, hall_par  &hall_params, qei_velocity_par &qei_velocity_params, int &avg_times, int max, chanend c_hall, chanend c_qei, qei_par &qei_params)
{
	int s, actual_velocity;
	int samples = avg_times;
	printintln(samples);printstr(" sens ");printintln(sensor_select);
	wait_ms(150, core_id, t);
	actual_velocity = get_average_velocity(sensor_select, c_hall, hall_params, qei_velocity_params, core_id, t, samples, c_qei, qei_params);
	if(actual_velocity >= max)
		s = 1;
	else
		s = 0;
	return {s, actual_velocity};
}

void commutation_sine_automate(int &sensor_select, chanend c_signal, chanend c_commutation, commutation_par &commutation_params,\
		hall_par &hall_params, qei_par &qei_params, chanend c_hall, chanend c_qei)
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
	qei_velocity_par qei_velocity_params;
	init_qei_velocity_params(qei_velocity_params);
	//xscope_initialise_1();
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

	{s1,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, qei_velocity_params, avg_times, (max_nominal_speed*75)/1000, c_hall, c_qei, qei_params);

	printintln(s1);
	//printintln(actual_velocity);

	comm_voltage = (comm_max * 375 )/1000;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);

	{s2,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, qei_velocity_params, avg_times, (max_nominal_speed*375)/1000, c_hall, c_qei, qei_params);
	//printintln(actual_velocity);
	printintln(s2);

	comm_voltage = comm_max ;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);

	{s3,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, qei_velocity_params, avg_times, max_reach_expected, c_hall, c_qei, qei_params);
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
	if(commutation_params.flag == 0)
	{
		commutation_params.max_speed_reached = max_reached_speed_pos;
		set_commutation_params( c_commutation, commutation_params);
	}


	comm_voltage = -(comm_max * 75 )/1000;
	i = 0;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_average_velocity(sensor_select, c_hall, hall_params, qei_velocity_params, core_id, t, avg_times, c_qei, qei_params);
	//printintln(actual_velocity);
	if(actual_velocity <= - (max_nominal_speed*75)/1000)
		s4 = 1;
	else
		s4 = 0;
	printintln(s4);

	comm_voltage = -(comm_max * 375 )/1000;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_average_velocity(sensor_select, c_hall, hall_params, qei_velocity_params, core_id, t, avg_times, c_qei, qei_params);
	//printintln(actual_velocity);
	if(actual_velocity <= - (max_nominal_speed*375)/1000)
		s5 = 1;
	else
		s5 = 0;
	printintln(s5);

	comm_voltage = - comm_max;
	ramp_down(i, comm_voltage, t, core_id, c_commutation);

	wait_ms(150, core_id, t);
	actual_velocity = get_average_velocity(sensor_select, c_hall, hall_params, qei_velocity_params, core_id, t, avg_times, c_qei, qei_params);
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

	if(commutation_params.flag == 0)
	{
		commutation_params.max_speed_reached = avg_speed_reach;
		set_commutation_params( c_commutation, commutation_params);//setting parameters
	}


	printstrln("test ended");
	//printintln(ok_positive);printstr(" ");printintln(ok_negative);

	i = 0;
	comm_voltage = comm_max ;
	ramp_up(i, comm_voltage, t, core_id, c_commutation);
	wait_ms(150, core_id, t);
	{s3,actual_velocity} = update_comm_sine_max_state(sensor_select, t, core_id, hall_params, qei_velocity_params, avg_times, max_reach_expected, c_hall, c_qei, qei_params);
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
	actual_velocity = get_average_velocity(sensor_select, c_hall, hall_params, qei_velocity_params, core_id, t, avg_times, c_qei, qei_params);
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
