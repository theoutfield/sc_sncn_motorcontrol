/**
 * @file hall_service.h
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once


#ifdef __XC__

#include <position_feedback_service.h>

/**
 *
 * @brief Service to read and process data from a Feedback Hall Sensor.
 *
 * @param hall_ports Ports structure defining where to read the Hall signals.
 * @param position_feedback_config Configuration for the service.
 * @param i_shared_memory Client interface to write to the shared memory.
 * @param i_hall Array of communication interfaces to handle up to 3 different clients.
 */
void hall_service(HallPorts &hall_ports, port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
                  client interface shared_memory_interface ?i_shared_memory,
                  server interface PositionFeedbackInterface i_position_feedback[3]);

#endif
