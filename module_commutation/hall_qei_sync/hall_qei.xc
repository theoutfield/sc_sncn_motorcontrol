#include "hall_qei.h"



int get_sync_position (chanend sync_output)
{
	int sync_position;

	sync_output <: 20;
	sync_output :> sync_position;

	return sync_position;
}

void calib_qei(int select_direction, commutation_par &commutation_params, timer t, chanend c_calib)
{
	unsigned int time;
	int calib_offset;
	int flag, flag_sent = 0;
	if(select_direction == 1) //fw
		c_calib <: 30;
	else if(select_direction == 2) //bw
		c_calib <: 40;
	t :> time;

	while(1)
	{
		t when timerafter(time + 10*MSEC_STD) :> time;

		if(select_direction == 1)
		{
			c_calib <: 50;
			flag_sent = 1;
		}
		else if(select_direction == 2)
		{
			c_calib <: 60;
			flag_sent = 1;
		}
		else
			flag_sent = 0;

		if(flag_sent == 1)
		{
			c_calib :> flag;
			c_calib :> calib_offset;
			if(flag == 0)
			{
				if(select_direction == 1)
					commutation_params.qei_forward_offset = calib_offset/100;
				else if(select_direction == 2)
					commutation_params.qei_backward_offset = calib_offset/100;
				break;
			}
		}
	}
	return;
}

void set_qei_offset(commutation_par &commutation_params, chanend c_calib)
{
//	printstrln("start");
	c_calib <: 70;
	c_calib <: commutation_params.qei_forward_offset;
	c_calib <: commutation_params.qei_backward_offset;
	return;
}

/**
 * \brief Calculates a synchronised position out of hall and qei
 *
 * \channel c_qei qei position data
 * \channel c_hall1 hall position data
 * \channel sync_output synchronised data from hall and qei
 */
void hall_qei_sync(qei_par &qei_params, hall_par &hall_params, commutation_par &commutation_params, chanend c_qei, chanend c_hall, chanend sync_output, chanend c_calib)
{
	int cmd; // Command token

	int hall_position = 0; // Hall input
	int qei_position = 0; // Qei input
	int sync_position = 0; // output

	int qei_valid; // qei validity (0 or 1)

	int qei_max_position = qei_params.max_count;
	int qei_crossover = qei_max_position - qei_max_position /10;
	timer t_qei, t_hall;
	unsigned int time_qei;
	unsigned int time_hall;

	int previous_position = 0;

	int max_count = qei_params.real_counts / hall_params.pole_pairs ;

	int diffi;

	int init = 1;

	int not_synced = 0;

	int calibrated_fw = 0;
	int calibrated_bw = 0;
	int direction = 0;
	int dummy;
	int condd= 1000;

	int times_no = 100;
	int calib_bw_offset = 0;
	int calib_fw_offset = 0;
	int calib_bw_flag = 0;
	int calib_fw_flag = 0;

	t_qei	:> time_qei;
	t_hall :> time_hall;
//	t_qei when timerafter(time_qei+ 7*SEC_STD) :> time_qei;

	while(1)
	{
		/* select loop
		 * 				reading qei at 3 micro seconds with case timer t_qei
		 * 				reading hall at 10 micro second with case timer t_hall
		 * sends out synchronized output over channel sync_output
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
						diffi = qei_position - previous_position;

						if( diffi > qei_crossover )
						{
							if(calib_bw_flag == 1 )
							{
								if(times_no >0)
								{
									calib_bw_offset += get_hall_position( c_hall);
									times_no--;
								}
								else
									//printintln(calib_bw_offset);
									calib_bw_flag = 0;
							}
							sync_position = commutation_params.qei_backward_offset;

						//	xscope_probe_data(3, condd);
							calibrated_bw = 1;
						}
						else if(diffi < -(qei_crossover))  // qei_max_position - 300
						{
							if(calib_fw_flag == 1 )
							{
							//	printstrln("ini fw loop");
							//	printintln(times_no);
								if(times_no >0)
								{
									calib_fw_offset += get_hall_position( c_hall);
									times_no--;
								}
								else
									//printintln(calib_fw_offset);
									calib_fw_flag = 0;
							}
							//if( ! (sync_position < 1596 - 20 && sync_position > 1596 + 20) )
							//{
							sync_position = commutation_params.qei_forward_offset;
							calibrated_fw = 1;
							//}
							//xscope_probe_data(3, condd);
						}
						else if( diffi < 10 && diffi > 0 )
						{
							sync_position = sync_position + diffi;
						}
						else if( diffi < 0 && diffi > -10)
						{
							sync_position = sync_position + diffi;
							if(sync_position < 0)
							{
								sync_position = max_count + sync_position;
							}
						}
						previous_position = qei_position;
					}
					if(sync_position >= max_count )
					{
						sync_position = 0;
					}
				}
				break;

//			case t_hall when timerafter(time_hall + 5000) :> time_hall: //4khz  20000 14000
//			hall_position = get_hall_position( c_hall);
//				xscope_probe_data(2, hall_position);
//			xscope_probe_data(1, sync_position);
//			xscope_probe_data(0, qei_position);
//			break;

			case sync_output :> cmd:
				if(cmd == 20)
				{
					if(direction == 1)
					{
						if(calibrated_fw == 1)
						{
							sync_output <: sync_position;
						}
						else
						{
							hall_position = get_hall_position( c_hall);
							sync_output <: (hall_position * max_count) >> 12;
						}
					}
					else if(direction == -1)
					{
						if(calibrated_bw == 1)
						{
							sync_output <: sync_position;
						}
						else
						{
							hall_position = get_hall_position( c_hall);
							sync_output <: (hall_position * max_count) >> 12;
						}
					}
					else
					{
						hall_position = get_hall_position( c_hall);
						sync_output <: (hall_position * max_count) >> 12;
					}
				}
				break;

			case c_calib :> cmd:
				if(cmd == 30)// initiate fw_calib
				{
					times_no = 100;
					calib_fw_flag = 1;
					calib_fw_offset = 0;
				}
				else if(cmd == 40) // initiate bw_calib
				{
					times_no = 100;
					calib_bw_flag = 1;
					calib_bw_offset = 0;
				}
				else if(cmd == 50)
				{
					c_calib <: calib_fw_flag;
					c_calib <: calib_fw_offset;
				}
				else if(cmd == 60)
				{
					c_calib <: calib_bw_flag;
					c_calib <: calib_bw_offset;
				}
				else if(cmd == 70)
				{
					//printintln(cmd);
					c_calib :> commutation_params.qei_forward_offset;
					c_calib :> commutation_params.qei_backward_offset;
					calibrated_bw = 0;
					calibrated_fw = 0;
					//printintln(commutation_params.qei_forward_offset);
					//printintln(commutation_params.qei_backward_offset);
				}
				break;
		}
	}
}

