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
#include "dc_motor_config.h"
#include "hall_config.h"
#include <xs1.h>
#include <stdint.h>

/**
 * \channel c_hall for communicating with the Hall Server
 *
 *  Output
 * \return the position in the range [0 - 4095] which maps to [0 - 359]/pole-pairs
 */
int get_hall_position(chanend c_hall);

/**
 * \channel c_hall for communicating with the Hall Server
 *
 *  Output
 * \return the counted up position (accounts for pole-pairs and gear-ratio)
 * 							in the range [0 - 4095]*pole-pairs*gear-ratio
 */
{int , int} get_hall_position_absolute(chanend c_hall);

/**
 * \brief Client library function for Hall Sensor
 *
 * \channel c_hall for communicating with the Hall Server
 *
 *  Input
 * \param hall_params struct defines the pole-pairs and gear ratio
 *
 *  Output
 * \return the velocity in rpm from Hall Sensor
 */
int get_hall_velocity(chanend c_hall, hall_par &hall_params);

