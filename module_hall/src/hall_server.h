/**
 * \file hall_server.h
 *
 *	Hall Sensor Server
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
 * below. The modifications to the code arse still covered by the
 * copyright notice above.
 *
 **/


#pragma once

#include <xs1.h>
#include <dc_motor_config.h>

/** \brief A basic hall encoder server
 *
 *  This implements the basic hall sensor server
 *
 *  \param c_hall the control channel for sending out hall position information
 *  \param p_hall the port for reading the hall sensor data
 */
void run_hall_new( chanend c_hall, chanend sensor_output, port in p_hall, hall_par &h_pole);

