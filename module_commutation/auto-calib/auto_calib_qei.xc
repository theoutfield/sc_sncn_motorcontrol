#include <auto_calib_qei.h>

void calculate_offset(qei_par &qei_params, hall_par &hall_params, chanend c_qei, chanend c_hall, timer t_qei, int &calib_bw_offset, int &calib_fw_offset, int calib_bw_flag, int calib_fw_flag)
{
	int hall_position = 0; // Hall input
		int qei_position = 0; // Qei input
		int sync_position = 0; // output

		int qei_valid; // qei validity (0 or 1)

		int qei_max_position = qei_params.max_count;
		int qei_crossover = qei_max_position - qei_max_position /10;
		unsigned int time_qei;

		int previous_position = 0;

		int max_count = qei_params.real_counts / hall_params.pole_pairs ;

		int difference;

		int init = 1;

		int not_synced = 0;

		int calibrated_fw = 0;
		int calibrated_bw = 0;
		int direction = 0;
		int dummy;
		int condd= 1000;

		int times_no = 100;

	t_qei	:> time_qei;
	while(1)
	{
		/* select loop
		 * 				reading qei at 3 micro seconds with case timer t_qei
		 * 				reading hall at 10 micro second with case timer t_hall
		 * calculate offset betwwen qei and hall
		 */
#pragma ordered
		select
		{
			case t_qei when timerafter(time_qei + 750) :> time_qei:
				{qei_position, qei_valid} = get_qei_position(c_qei, qei_params); //aquisition
				{dummy, direction}= get_qei_position_absolute(c_qei);
				if(qei_valid==1)
				{
					qei_position = qei_max_position - qei_position;
					/* Runs only once*/
					if(init == 1)
					{
						previous_position = qei_position;
						init=0;
					}
					if(previous_position!= qei_position )
					{
						difference = qei_position - previous_position;

						if( difference > qei_crossover )
						{
							if(calib_bw_flag == 1 )
							{
								if(times_no >0)
								{
									calib_bw_offset += get_hall_position( c_hall);
									times_no--;
								}
								else
									calib_bw_flag = 0;
							}
							calibrated_bw = 1;
						}
						else if(difference < -(qei_crossover))
						{
							if(calib_fw_flag == 1 )
							{
								if(times_no >0)
								{
									calib_fw_offset += get_hall_position( c_hall);
									times_no--;
								}
								else
									calib_fw_flag = 0;
							}
							calibrated_fw = 1;
						}
						previous_position = qei_position;
					}
				}
				break;
		}
		if(times_no == 0)
			break;
	}
}

void qei_calibrate(chanend c_signal, chanend c_commutation, commutation_par &commutation_params,\
		hall_par &hall_params, qei_par &qei_params, chanend c_hall, chanend c_qei, chanend c_calib) //commutation purpose also send the offset to the thread.
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
	int comm_min = 2500;
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
	int speed;

	timer t_qei;
	int calib_bw_offset = 0;
	int calib_fw_offset = 0;
	int calib_bw_flag = 0;
	int calib_fw_flag = 0;
	int times_no = 100;

	init_qei_velocity_params(qei_velocity_params);
	while(1)
	{

		init_state = __check_commutation_init(c_signal);
		if(init_state == INIT)
		{
			printstrln("commutation intialized");
			break;
		}
	}

	i = 0;
	ramp_up(i, comm_min, t, core_id, c_commutation); // fw

	calib_fw_flag = 1;
	printstrln("loop start");
	printintln(times_no);

	calculate_offset(qei_params, hall_params, c_qei, c_hall, t_qei, calib_bw_offset, calib_fw_offset, calib_bw_flag, calib_fw_flag);

	//calib_qei(1, commutation_params, t, c_calib);
	commutation_params.qei_forward_offset = ((calib_fw_offset*qei_params.real_counts)/(4096*hall_params.pole_pairs))/times_no;
	//calib_fw_offset = calib_qei(c_calib,  t, 1);
	printintln(commutation_params.qei_forward_offset);

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


	ramp_down(i, -comm_min, t, core_id, c_commutation);

	calib_bw_flag = 1;
	printstrln("loop start");

	//calib_qei(2, commutation_params, t, c_calib);
	calculate_offset(qei_params, hall_params, c_qei, c_hall, t_qei, calib_bw_offset, calib_fw_offset, calib_bw_flag, calib_fw_flag);


	commutation_params.qei_backward_offset = ((calib_bw_offset*qei_params.real_counts)/(4096*hall_params.pole_pairs))/times_no;
	printintln(commutation_params.qei_backward_offset);

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

	set_qei_sync_offset(c_qei, commutation_params.qei_forward_offset, commutation_params.qei_backward_offset);


	commutation_params.offset_forward = 682;
	set_commutation_params(c_commutation, commutation_params);
	commutation_sensor_select( c_commutation, 2); //QEI

	i = 0;
	ramp_up(i, 1000, t, core_id, c_commutation); // fw

	wait_ms(5000, core_id, t);
/*
	//printintln(get_hall_velocity(c_hall, hall_params));
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

	ramp_down(i, -1000, t, core_id, c_commutation);
	wait_ms(5000, core_id, t);
	//printintln(get_hall_velocity(c_hall, hall_params));
	if(i<0)
		sense = 1;
	else if(i>0)
		sense = -1;
	while( i != 0 )
	{
		set_commutation_sinusoidal(c_commutation, i);
		i = (i + sense * 10);
		wait_ms(5, core_id, t);
	}*/
}
