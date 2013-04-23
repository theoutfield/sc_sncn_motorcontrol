#include <xs1.h>
#include <stdint.h>
#include "hall_client.h"
#include "refclk.h"
#include "dc_motor_config.h"



int get_hall_angle(chanend c_hall) {
	int pos;
	c_hall	<: 1;
	slave
	{
		c_hall :> pos;  // angle
	}

	return pos;
}

int get_hall_speed(chanend c_hall) {
	int speed;
	c_hall	<: 2;
	slave
	{
		c_hall :> speed; // speed
	}

	return speed;
}

{int, int} get_hall_values(chanend c_hall) {
	int pos, speed;
	c_hall	<: 3;
	slave
	{
		c_hall :> pos;
		c_hall :> speed;
	}

	return {speed, pos};
}




