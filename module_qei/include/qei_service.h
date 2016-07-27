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
 * @param qei_ports Ports structure defining where to access the Encoder signals.
 * @param qei_config Configuration for the service.
 * @param i_qei Array of communication interfaces to handle up to 5 different clients.
 */
void qei_service(BISSPorts &qei_ports, PositionFeedbackConfig &position_feedback_config,
                 client interface shared_memory_interface ?i_shared_memory,
                 server interface PositionFeedbackInterface i_position_feedback[3]);

#endif
