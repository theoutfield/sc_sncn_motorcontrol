/**
 * \file hall_client.h
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
#pragma once
#include <stdint.h>

/* Get speed in rpm*/
int32_t get_hall_speed(chanend c_hall);

/* Get angle in range 0 - 4095 maps to 0 - 360 degrees*/
int32_t get_hall_angle(chanend c_hall);


