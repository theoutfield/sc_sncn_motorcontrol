
/**
 * \file delay.xc
 * \brief Implementation of delay functions
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#include <refclk.h>

void wait_micro_s(int microseconds, int tile_id, timer t)
{
	unsigned int time;
	t :> time;
	if(tile_id == 3)
	{
		t when timerafter(time + (microseconds * MSEC_FAST)/1000) :> time;
	}
	else
	{
		t when timerafter(time + (microseconds * MSEC_STD)/1000) :> time;
	}
	return;
}

void wait_ms(int milliseconds, int tile_id, timer t)
{
	unsigned int time;
	t :> time;
	if(tile_id == 3)
	{
		t when timerafter(time + milliseconds * MSEC_FAST) :> time;
	}
	else
	{
		t when timerafter(time + milliseconds * MSEC_STD) :> time;
	}
	return;
}

void wait_s(int seconds, int tile_id, timer t)
{
	unsigned int time;
	t :> time;
	if(tile_id == 3)
	{
		t when timerafter(time + seconds * SEC_FAST) :> time;
	}
	else
	{
		t when timerafter(time + seconds * SEC_STD) :> time;
	}
	return;
}
