/**
 * Module:  module_dsc_hall
 * Version: 1v0alpha2
 * Build:   60a90cca6296c0154ccc44e1375cc3966292f74e
 * File:    hall_client.xc
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/                                   
#include <xs1.h>
#include <stdint.h>
#include "hall_input.h"
#include "refclk.h"
#include "dc_motor_config.h"

{int, int, int, int} get_hall_values(chanend c_hall)
{
int speed;
int angle;
int position;
int pinstate;

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


{int, int, int, int} get_encoder_values(chanend c_hall)
{
	int speed;
	int angle;
	int position;
	int pinstate;

	c_hall <: 2;
	slave
	{
	c_hall :> speed;
	c_hall :> angle;
	c_hall :> position;
	c_hall :> pinstate;
	}
	return { speed, angle, position, pinstate };
}


{int, int, int, int, int, int, int, int, int, int} get_info_hall_input(chanend c_hall)
{
	int a1,a2,a3,a4,a5,a6,a7,a8,a9,a10;

	c_hall <: 3;
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



