/**
 * \file hall_server.h
 *
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



/** \brief A basic hall encoder server
 *
 *  This implements the basic hall sensor server
 *
 *  \param c_hall the control channel for reading hall position
 *  \param h_pole defines the pole-pairs for the hall sensor
 *  \param p_hall the port for reading the hall sensor data
 */
void run_hall( port in p_hall, chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4);
