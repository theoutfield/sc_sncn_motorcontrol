/**
 * \file hall_server.h
 * \brief Hall Sensor Server Implementation
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
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
#include "hall_config.h"

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

/** \brief A basic hall sensor server
 *
 * \Output
 * \param c_hall_p1 the control channel for reading hall position in order of priority (highest) 1 ... (lowest) 5
 * \param c_hall_p2 the control channel for reading hall position (priority 2)
 * \param c_hall_p3 the control channel for reading hall position (priority 3)
 * \param c_hall_p4 the control channel for reading hall position (priority 4)
 * \param c_hall_p5 the control channel for reading hall position (priority 5)
 *
 * \Input
 * \param p_hall the port for reading the hall sensor data
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void run_hall(chanend c_hall_p1, chanend c_hall_p2, chanend c_hall_p3, chanend c_hall_p4,
              chanend c_hall_p5, chanend c_hall_p6, port in p_hall, hall_par &hall_params);

