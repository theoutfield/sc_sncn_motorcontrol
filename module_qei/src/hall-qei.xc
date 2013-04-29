#include "hall-qei.h"
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
/**
 *  \channel c_qei qei position data
 *  \channel c_hall1 hall position data
 *  \channel synchronised data from hall and qei
 */
void hall_qei_sync(chanend c_qei, chanend c_hall1, chanend sync_output)
{
	int hall_pos, qei_pos; // input
	int sync_pos; // output

	int speed, v;

	int max = 4095;
	unsigned int t1, t2;
	timer t_qei, t_hall;

	signed pos, spos, prev = 0, count = 0, co12 = 0, first = 1, set = 0;
	int max_count = QEI_COUNTS / POLE_PAIRS;//GEAR_RATIO * QEI_COUNTS;
	int enc_div = POLE_PAIRS * GEAR_RATIO;
	int hall_p;
	int diffi, count1, cmd;

	int forw_ratio = 4096 / max_count;
	int angle_qei, diff_angle_qei, diff_angle;

	int gaurd = 7;

	int s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0;

	int not_synced = 0;
	t_qei:> t1;
	t_hall :> t2;
	t_qei when timerafter(t1+ 7*SEC_STD) :> t1;


	while(1)
	{

		// select loop
		// 				reading qei at 3 micro seconds with case timer t_qei
		// 				reading hall at 10 micro second with case timer t_hall
		// sends out synchronized output over channel sync_output
		select
		{
			case t_qei when timerafter(t1 + 300) :> t1:
				{	qei_pos, v}= get_qei_pos( c_qei); //acq     //{speed, qei_pos, v} = get_qei_data(c_qei);
				qei_pos = max - qei_pos;

				if(first==1 )
				{
					prev = qei_pos;
					first=0;
					set = prev;
				}
				if(prev!= qei_pos )
				{
					spos=qei_pos;
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
						if(count < 0)
						{
							count = max_count + count;
						}
					}
					prev=spos;
					if(prev==set)
					co12++;
				}
				if(count >= max_count)
				{
					co12=0;
					count=0;
				}

				//xscope_probe_data(1, count);
			break;

			case t_hall when timerafter(t2 + 1000) :> t2: //4khz  20000 14000
				hall_pos = get_hall_angle( c_hall1);
				//xscope_probe_data(0, hall_pos);

				if( hall_pos >= 681 && hall_pos < 682+gaurd )
				{
					s1++;
					s6 = 0;
					s2 = 0;

					if(s1 < 3 || not_synced <= 4 )
					{
						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);

						if(count <0 )
						count = max_count + count;

						if(not_synced <= 4)
						not_synced++;
					}

				}
				else if( hall_pos >= 1364 && hall_pos < 1365+gaurd )
				{
					s2++;
					s1 = 0;
					s3 = 0;

					if(s2 < 3 || not_synced <= 4) {

						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);
						if(count <0 )
						count = max_count + count;
						if(not_synced <= 4)
						not_synced++;
					}
				}
				else if( hall_pos >= 2047 && hall_pos < 2048+gaurd )
				{
					s3++;
					s2 = 0;
					s4 = 0;
					if(s3<3 || not_synced <= 4) {

						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);
						if(count <0 )
						count = max_count + count;
						if(not_synced <= 4)
						not_synced++;
					}
				}
				else if( hall_pos >= 2729 && hall_pos < 2730+gaurd )
				{
					s4++;
					s3 = 0;
					s5 = 0;

					if(s4< 3 || not_synced <= 4) {
						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);
						if(count <0 )
						count = max_count + count;
						if(not_synced <= 4)
						not_synced++;
					}
				}
				else if( hall_pos >= 3414 && hall_pos < 3413+gaurd )
				{
					s5++;
					s4 = 0;
					s6 = 0;
					if(s5<3 || not_synced <= 4) {
						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);
						if(count <0 )
						count = max_count + count;
						if(not_synced <= 4)
						not_synced++;
					}
				}
				else if( hall_pos >= 4090 ) //( hall_pos >= 4095 || hall_pos < gaurd )
				{
					s6++;
					s5 =0 ;
					s1= 0;
					if(s6<3 || not_synced <= 4) {
						angle_qei = (count<<12)/500;
						diff_angle = hall_pos - angle_qei;

						diff_angle_qei = (diff_angle*500)>>12;

						count = (diff_angle_qei + count);
						if(count <0 )
						count = max_count + count;
						if(not_synced <= 4)
						not_synced++;
					}
				}

				break;

				case sync_output :> cmd:
				if(cmd == 20)
				{
					if(not_synced<3)
						sync_output <: hall_pos;
					else
					{
						sync_output <: count;

					}

				}
			break;
		}

	}
}
