
/**
 * \file hall_server.h
 * \brief Hall Sensor Server Implementation
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
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

#pragma once

#include <xs1.h>
#include <bldc_motor_config.h>
#include <filter_blocks.h>
#include <refclk.h>
#include <stdlib.h>
#include <print.h>
#include <stdint.h>
#include "refclk.h"
#include <internal_config.h>
#include "hall_config.h"
#include <xscope.h>


/** \brief A basic hall encoder server
 *
 *  This implements the basic hall sensor server
 *
 *	Output channel
 * 	\channel c_hall_p1 the control channel for reading hall position in order of priority (highest) 1 ... (lowest) 5
 *  \channel c_hall_p2 the control channel for reading hall position (priority 2)
 *  \channel c_hall_p3 the control channel for reading hall position (priority 3)
 *  \channel c_hall_p4 the control channel for reading hall position (priority 4)
 *  \channel c_hall_p5 the control channel for reading hall position (priority 5)
 *
 *	Input port
 *	\port p_hall the port for reading the hall sensor data
 *
 *	 Input
 *	\param hall_params struct defines the pole-pairs and gear ratio
 */
void run_hall(chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4, \
		chanend c_hall_p5, chanend c_hall_p6, port in p_hall, hall_par &hall_params);
//void run_hall(chanend c_hall[6], port in p_hall, hall_par &hall_params);
