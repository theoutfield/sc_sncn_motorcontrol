/**
 * \file hall_client.xc
 *
 *	Hall Sensor Client
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and Synapticon GmbH.
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright 2013, Synapticon GmbH & XMOS Ltd. All rights reserved.
 * Authors: Martin Schwarz <mschwarz@synapticon.com>
 *
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/
#include <xs1.h>
#include <stdint.h>
#include "hall_client.h"
#include "refclk.h"
#include "dc_motor_config.h"


int32_t get_hall_speed(chanend c_hall)
{
  int32_t speed;
  c_hall <: 2;
  c_hall :> speed;  			// speed
  
  return speed;
}

int32_t get_hall_angle(chanend c_hall)
{
  int32_t pos;
  c_hall <: 1;
  c_hall :> pos;  				// 6 steps angle

  return pos;
}


int hall_enc_const = POLE_PAIRS * GEAR_RATIO * 3600;


