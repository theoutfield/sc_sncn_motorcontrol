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


