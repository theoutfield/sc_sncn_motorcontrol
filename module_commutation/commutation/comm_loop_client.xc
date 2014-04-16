
/**
 * \file comm_loop_client.xc
 * \brief Commutation Loop Client functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Commutation Loop Client functions
 *
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#include "comm_loop_client.h"
#include <xs1.h>
#include <stdint.h>
#include <bldc_motor_config.h>
#include "refclk.h"
#include <internal_config.h>
#include "print.h"


void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed)
{
	commutation_params.angle_variance = 1024/(hall_params.pole_pairs * 3); // (60 * 4096)/( POLE_PAIRS * 2 *360)
	if(hall_params.pole_pairs < 4)
	{
		commutation_params.max_speed_reached = nominal_speed*4;
		commutation_params.flag = 1;
	}
	else if(hall_params.pole_pairs >=4)
	{
		commutation_params.max_speed_reached = nominal_speed;//10000;//
		commutation_params.flag = 0;
	}
	commutation_params.hall_offset_clk =  COMMUTATION_OFFSET_CLK;
	commutation_params.hall_offset_cclk = COMMUTATION_OFFSET_CCLK;
	commutation_params.winding_type = WINDING_TYPE;
	commutation_params.qei_forward_offset = 0;
	commutation_params.qei_backward_offset = 0;
//	printintln(commutation_params.angle_variance);
//	printintln(commutation_params.max_speed_reached);
}


void commutation_sensor_select(chanend c_commutation, int sensor_select)
{
	c_commutation <: SENSOR_SELECT;
	c_commutation <: sensor_select;
	return;
}

void set_commutation_params(chanend c_commutation, commutation_par &commutation_params)
{
	c_commutation <: SET_COMMUTATION_PARAMS;
	c_commutation :> commutation_params.angle_variance;
	c_commutation :> commutation_params.max_speed_reached;
	c_commutation :> commutation_params.hall_offset_clk;
	c_commutation :> commutation_params.hall_offset_cclk;
	c_commutation :> commutation_params.winding_type;
}

/* MAX Input value 13739 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage)
{
	c_commutation <: 2;
	c_commutation <: input_voltage;
	return;
}
