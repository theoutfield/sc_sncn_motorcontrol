/**
 * \file hall_client.h
 *
 *	Hall Sensor Client
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
#include <stdint.h>

/**
 * The client library function for hall sensor server
 *
 * \channels:
 * 			c_hall -	the channel for communicating with the hall server
 */

/* function returns the speed in rpm from Hall Sensor*/
int get_hall_speed(chanend c_hall);

/* function returns the angle in range [0 - 4095] which maps to [0 - 359] degrees*/
int get_hall_angle(chanend c_hall);

/* function returns the angle and speed respectively from the Hall Sensor*/
{int, int} get_hall_values(chanend c_hall);


