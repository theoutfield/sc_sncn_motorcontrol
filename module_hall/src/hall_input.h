/**
 * \file hall_input.h
 *
 *	Hall Sensor Server
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and Synapticon GmbH.
 *
 * Copyright 2013, Synapticon GmbH & XMOS Ltd. All rights reserved.
 * Authors:  Martin Schwarz <mschwarz@synapticon.com> &  Ludwig Orgler <orgler@tin.it>
 *
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code arse still covered by the
 * copyright notice above.
 *
 **/

#pragma once

#include <xs1.h>
#include <dc_motor_config.h>

/*
 * debug uart function
 */
void run_uart(chanend c_motvalue, clock clk1);

/** \brief Hall Encoder Server
 *
 *  This implements the basic hall sensor server
 *
 *  \channels:
 * 				c_hall 		- 	the control channel for sending out hall position
 *  \ports:
 *  			p_hall 		-	port for reading the hall sensor data
 *  		    p_encoder 	-	port for reading the quadrature encoder data
 */
void run_hall(chanend c_hall, port in p_hall, port in p_encoder);

