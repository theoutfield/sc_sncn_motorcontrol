#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"
#include <bldc_motor_config.h>
#include <qei_client.h>
#include <qei_server.h>

void qei_unit_test()
{
	int max, min;
	qei_par qei_params;
	in_data d;
	while(1)
	{
		input_max_position(d);
		input_min_position(d);
//		init_qei_param(qei_params, d.max_position, d.min_position);
		printstr("\n Max ticks: ");printintln(qei_params.max_ticks);
	}

}

void qei_hall_example(chanend c_qei, chanend c_hall)
{
	int position;
	int velocity;
	int valid;
	int core_id = 1;
	timer t;
	int index_count;
	int count=0;
	qei_par qei_params;
	qei_velocity_par qei_velocity_params;  // to compute velocity from qei
	//hall_par hall_params;
	int hall_p, hall_di, hall_velocity, time;
//	init_hall_param(hall_params);
	init_qei_param(qei_params);
	init_qei_velocity_params(qei_velocity_params);	// to compute velocity from qei

#ifdef ENABLE_xscope_main
	xscope_initialise_1();
#endif

	t :> time;
	while(1)
	{{position, valid} =get_qei_position_absolute(c_qei); 	//
		//{position, valid} = get_qei_position(c_qei, qei_params);		//		//hall_velocity =  get_hall_velocity(c_hall, hall_params);//velocity = get_qei_velocity(c_qei, qei_params, qei_velocity_params);
		//{hall_p, hall_di} =  get_hall_position_absolute(c_hall);
		//{index_count, position, count} = get_qei_data(c_qei);
		wait_ms(1, core_id, t);			//t when timerafter(time+13000) :> time;
	#ifdef ENABLE_xscope_main
		xscope_probe_data(0, position);
		xscope_probe_data(1, hall_p);
		//xscope_probe_data(2, hall_velocity);
		//xscope_probe_data(1, velocity);
	#else
		printstr("Position: ");
		printint(position);
		//printstr(" ");
		//printstr("index: "); // with print velocity information will be corrupt (use xscope)
		//printint(position&(qei_params.max_ticks_per_turn-1));
		printstr(" ");
			printstr("count: "); // with print velocity information will be corrupt (use xscope)
			printintln(count);
	#endif
	}
}
