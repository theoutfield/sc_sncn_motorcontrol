
/**
 * \file hall_client.h
 * \brief Hall Sensor Client Functions
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

#pragma once
#include <stdint.h>
#include <bldc_motor_config.h>
#include "hall_config.h"
#include <xs1.h>


/**
 * \brief Get position from Hall Server
 *
 *  Output channel
 * \channel c_hall for communicating with the Hall Server
 *
 *  Output
 * \return the position in the range [0 - 4095] which maps to [0 - 359]/pole-pairs
 */
int get_hall_position(chanend c_hall);

/**
 * \brief Get absolute position from Hall Server
 *
 * Output channel
 * \channel c_hall for communicating with the Hall Server
 *
 *  Output
 * \return the counted up position (compensates for pole-pairs)
 * 		   in the range [0 - 4095] * pole-pairs
 */
{int , int} get_hall_position_absolute(chanend c_hall);

/**
 * \brief Get Velocity from Hall Server
 *
 * Output channel
 * \channel c_hall for communicating with the Hall Server
 *
 *  Input
 * \param hall_params struct defines the pole-pairs
 *
 *  Output
 * \return the velocity in rpm from Hall Sensor
 */
int get_hall_velocity(chanend c_hall, hall_par &hall_params);

void reset_hall_count(chanend c_hall, int offset);
