#include <xs1.h>
#include <stdint.h>
#include "hall_client.h"
#include "refclk.h"
#include "dc_motor_config.h"

{int, int, int, int} get_hall_values(chanend c_hall) {
	int speed;
	int angle;
	int position;
	int pinstate;

	c_hall	<: 1;
	slave
	{
		c_hall :> speed;
		c_hall :> angle;
		c_hall :> position;
		c_hall :> pinstate;
	}

	return {speed, angle, position, pinstate};
}

{int, int, int, int} get_encoder_values(chanend c_hall) {
	int speed;
	int angle;
	int position;
	int pinstate;

	c_hall	<: 2;
	slave
	{
		c_hall :> speed;
		c_hall :> angle;
		c_hall :> position;
		c_hall :> pinstate;
	}
	return {speed, angle, position, pinstate};
}

{int, int, int, int, int, int, int, int, int, int} get_info_hall_input(
		chanend c_hall) {
	int a1, a2, a3, a4, a5, a6, a7, a8, a9, a10;

	c_hall	<: 3;
	slave
	{
		c_hall :> a1;
		c_hall :> a2;
		c_hall :> a3;
		c_hall :> a4;
		c_hall :> a5;
		c_hall :> a6;
		c_hall :> a7;
		c_hall :> a8;
		c_hall :> a9;
		c_hall :> a10;
	}

	return {a1,a2,a3,a4,a5,a6,a7,a8,a9,a10};
}


int32_t get_hall_speed(chanend c_hall) {
	int32_t speed;
	c_hall	<: 2;
	c_hall :> speed; // speed

	return speed;
}

int32_t get_hall_angle(chanend c_hall) {
	int32_t pos;
	c_hall	<: 1;
	c_hall :> pos; // 6 steps angle

	return pos;
}

