
/**
 * \file hall_client.h
 * \brief Hall Sensor Client Functions
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
*/

#pragma once

#include <hall_config.h>
#include <xs1.h>

/**
 * \brief Gets position from Hall Server
 *
 * \Output
 * \param c_hall A chanend connected to Hall Server
 *
 * \return the position in the range [0 - 4095] which maps to [0 - 359]/pole-pairs
 */
int get_hall_position(chanend c_hall);

/**
 * \brief Gets absolute position from Hall Server
 *
 * \Output
 * \param c_hall A chanend connected to Hall Server
 *
 * \return the counted up position (compensates for pole-pairs)
 * 		   in the range [0 - 4095] * pole-pairs
 */
{int , int} get_hall_position_absolute(chanend c_hall);

/**
 * \brief Gets velocity from Hall Server
 *
 * \Output
 * \param c_hall A chanend connected to Hall Server
 *
 * \Input
 * \param hall_params Structure holding parameters like e.g. "number of pole-pairs"
 *
 * \return the velocity in rpm from Hall Sensor
 */
int get_hall_velocity(chanend c_hall, hall_par &hall_params);

void reset_hall_count(chanend c_hall, int offset);
