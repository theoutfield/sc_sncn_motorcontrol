#include "test.h"
#include <xs1.h>
#include <platform.h>
#include<print.h>
#include "refclk.h"

void set_torque_test(chanend c_torque) {
	int torque;
	in_data d;
	while (1) {
		input_torq(d);
		printintln(d.set_torque);

		c_torque <: 2;
		c_torque <: d.set_torque;

	}
}

/*{
	int voltage = 1500;
	//check init signal from commutation level
	while (1)
	{
		unsigned received_command = 0 , command;
		select
		{
			case c_signal :> command: 			//SIGNAL_READ(command):
				received_command = 1;
				break;
			default:
				break;
		}
		if(received_command == 1)
		{
			printstrln(" init commutation");
			break;
		}
	}

	while(1)
	{
		set_commutation_sinusoidal(c_commutation, voltage);
	}
}*/

/*{
	int pos, v;
	timer ts;
	unsigned time;
	ts:>time;
	while(1)
	{
		ts when timerafter(time+1000) :> time;
		{pos, v} = get_qei_position(c_qei );
		//printintln(pos);
		xscope_probe_data(0, pos);
		xscope_probe_data(1, v);
	}
}*/

/*
	{
		timer ts;
		unsigned time, a1, a2, init;

		ts:>time;
		init = init_commutation(c_signal);
			if(init == 1)
				printstrln("initialized commutation");
			else
				printstrln(" initialize commutation failed");

		while(1)
		{
			ts when timerafter(time+5556) :> time; // .05 ms
			{	a1 , a2}= get_adc_vals_calibrated_int16_ad7949(c_adc);
			xscope_probe_data(0, a1);
			xscope_probe_data(1, a2);
		}
	}
 */
