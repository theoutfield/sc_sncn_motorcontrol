
/**
 * \file comm_loop.h
 *
 *	Commutation Loop based on Space Vector PWM method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>, Ludwig Orgler <orgler@tin.it>
 * 			& Martin Schwarz <mschwarz@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "predriver/a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"

/**
 * \brief Commutation Loop
 *
 * \channel c_value channel to receive motor power input value
 * \channel c_pwm_ctrl channel to set pwm level output
 * \channel sig channel for signaling to start adc after initialization
 *
 */
void commutation(chanend c_value, chanend c_pwm_ctrl, chanend sig);
