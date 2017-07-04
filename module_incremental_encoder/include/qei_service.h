/**
 * @file qei_service.h
 * @brief Incremental Encoder Service Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#ifdef __XC__

#include <position_feedback_service.h>


/**
 * @brief Service to read and process data from an Feedback Incremental Encoder Sensor.
 *
 * @param qei_hall_port Port to read the Encoder signals.
 * @param gpio_ports GPIO ports array
 * @param position_feedback_config Configuration for the service.
 * @param i_shared_memory Client interface to write the position data to the shared memory.
 * @param i_position_feedback Server interface used by clients for configuration and direct position read.
 * @param gpio_on Set to 1 to enable GPIO read/write.
 */
void qei_service(port qei_hall_port, port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
                 client interface shared_memory_interface ?i_shared_memory,
                 server interface PositionFeedbackInterface i_position_feedback[3],
                 int gpio_on);

#endif
