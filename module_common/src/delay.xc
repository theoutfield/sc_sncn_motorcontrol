#include <refclk.h>

void wait_ms(int milliseconds, int core_id, timer t)
{
	unsigned time;
	t :> time;
	if(core_id == 3)
	{
		t when timerafter(time + milliseconds * MSEC_FAST) :> time;
	}
	else
	{
		t when timerafter(time + milliseconds * MSEC_STD) :> time;
	}
	return;
}

void wait_s(int seconds, int core_id, timer t)
{
	unsigned time;
	t :> time;
	if(core_id == 3)
	{
		t when timerafter(time + seconds * SEC_FAST) :> time;
	}
	else
	{
		t when timerafter(time + seconds * SEC_STD) :> time;
	}
	return;
}
