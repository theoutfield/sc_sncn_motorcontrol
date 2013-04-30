/**
 * Module:  module_enc_hall
 * Ludwig Orgler orgler@tin.it synapticon 04/2013
  *
 **/

#include <xs1.h>
#include <stdint.h>
#include "hall_server.h"
#include "refclk.h"
#include "dc_motor_config.h"

{int, int, int, int} get_hall_values(chanend c_hall)
{
int speed,angle,position,pinstate;
	c_hall <: 1;
	slave
	{
	c_hall :> speed;
	c_hall :> angle;
	c_hall :> position;
	c_hall :> pinstate;
	}
return { speed, angle, position, pinstate };
}


{int, int, int, int} get_encoder_values(chanend c_encoder)
{
int speed,angle,position,pinstate;
	c_encoder <: 1;
	slave
	{
	c_encoder :> speed;
	c_encoder :> angle;
	c_encoder :> position;
	c_encoder :> pinstate;
	}
	return { speed, angle, position, pinstate };
}



