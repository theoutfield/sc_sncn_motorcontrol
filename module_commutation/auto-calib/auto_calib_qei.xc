#include <auto_calib_qei.h>

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
	int comm_min = 5000;
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

	int calib_fw_offset;
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

	calib_qei(1, commutation_params, t, c_calib);
	commutation_params.qei_forward_offset = (commutation_params.qei_forward_offset*qei_params.real_counts)/(4096*hall_params.pole_pairs);
	//calib_fw_offset = calib_qei(c_calib,  t, 1);
	//printintln(commutation_params.qei_forward_offset);

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

	calib_qei(2, commutation_params, t, c_calib);
	commutation_params.qei_backward_offset = (commutation_params.qei_backward_offset*qei_params.real_counts)/(4096*hall_params.pole_pairs);
	//printintln(commutation_params.qei_backward_offset);

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
	set_qei_offset(commutation_params, c_calib);

}
