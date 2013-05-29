#include "hall-qei.h"
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "xscope.h"

/**
 * \brief Sincronizes 3 times at given step
 *
 * TODO: Get rid of this sync stuff in general
 *
 * \param s Array of sync steps
 * \param si Current sync step
 * \param sync_position
 * \param hall_position
 * \param not_synced
 * \param max_count
 * \return syncronised position
 */
{int, int} sync_it(int s[], int si, int sync_position, int hall_position,
		int not_synced, int max_count) {
	int angle_qei;
	int diff_angle;
	int diff_angle_qei;
	int i;
	// flush whole array
	for (i = 0; i < 6; ++i) {
		if(i != si) s[i] = 0;
	}


	s[si] = s[si] + 1;

	if (s[si] < 3 || not_synced <= 4) {
		angle_qei = (sync_position << 12) / 500;
		diff_angle = hall_position - angle_qei;

		diff_angle_qei = (diff_angle * 500) >> 12;

		sync_position = (diff_angle_qei + sync_position);

		if (sync_position < 0)
			sync_position = max_count + sync_position;

		if (not_synced <= 4)
			not_synced++;
	}

	return {sync_position, not_synced};
}


int get_sync_position ( chanend sync_output )
{
	int sync_position;

	sync_output <: 20;
	sync_output :> sync_position;

	return sync_position;
}
/**
 * \brief Calculates a synchronised position out of hall and qei
 *
 * \channel c_qei qei position data
 * \channel c_hall1 hall position data
 * \channel sync_output synchronised data from hall and qei
 */
void hall_qei_sync(chanend c_qei, chanend c_hall1, chanend sync_output) {

	int cmd; // Command token

	int hall_position = 0; // Hall input
	int qei_position = 0; // Qei input
	int sync_position = 0; // output

	int qei_valid; // qei validity (0 or 1)

	int hall_max_position = 4095;

	timer t_qei, t_hall;
	unsigned int time_qei;
	unsigned int time_hall;

	int previous_position = 0;

	int max_count = QEI_COUNT_MAX_REAL / POLE_PAIRS;

	int diffi;

	int gaurd = 7;

	int s[] = { 0, 0, 0, 0, 0, 0 };

	int init = 1;

	int not_synced = 0;

	int not_calibrated = 1;
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

			case sync_output :> cmd:
			if(cmd == 20)
			{
				if(not_synced<4)
					sync_output <: (hall_position * 500) >> 12;
				else
				{
					sync_output <: sync_position;

				}
			}
			break;

			case t_qei when timerafter(time_qei + 500) :> time_qei:
			{	qei_position, qei_valid}= get_qei_position( c_qei); //aquisition
			qei_position = hall_max_position - qei_position;

			if(qei_valid==1)
			{
				/* Runs only once*/
				not_calibrated = 0;
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
						sync_position = sync_position + diffi;
						if(sync_position < 0)
						{
							sync_position = max_count + sync_position;
						}
					}
					previous_position=qei_position;
				}
				if(sync_position >= max_count)
				{
					sync_position=0;
				}
				xscope_probe_data(0, sync_position);
			}

			break;

			case t_hall when timerafter(time_hall + 5000) :> time_hall: //4khz  20000 14000
			hall_position = get_hall_angle( c_hall1);
		//	xscope_probe_data(1, hall_position);



			if( hall_position >= 681 && hall_position < 682+gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 0, sync_position, hall_position, not_synced, max_count);

			}
			else if( hall_position >= 1364 && hall_position < 1365+gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 1, sync_position, hall_position, not_synced, max_count);

			}
			else if( hall_position >= 2047 && hall_position < 2048+gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 2, sync_position, hall_position, not_synced, max_count);

			}
			else if( hall_position >= 2729 && hall_position < 2730+gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 3, sync_position, hall_position, not_synced, max_count);

			}
			else if( hall_position >= 3414 && hall_position < 3413+gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 4, sync_position, hall_position, not_synced, max_count);

			}
			else if( hall_position >= 4093 || hall_position < gaurd )
			{

				{sync_position, not_synced} = sync_it(s, 5, sync_position, hall_position, not_synced, max_count);

			}

			break;


		}
	}
}
