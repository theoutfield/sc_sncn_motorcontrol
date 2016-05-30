/*
 * position_feedback_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <biss_service.h>
#include <contelec_service.h>
#include <position_feedback_service.h>

void position_feedback_service(PositionFeedbackPorts &position_feedback_ports, PositionFeedbackConfig &position_feedback_config,
                      client interface shared_memory_interface ?i_shared_memory,
                      server interface PositionFeedbackInterface i_position_feedback[3])
{
    for (int i=0; i<2; i++) {
        switch(position_feedback_config.sensor_type[i]) {
        case BISS_SENSOR:
            biss_service(position_feedback_ports.biss_ports, position_feedback_config.biss_config, i_shared_memory, i_position_feedback);
            break;
        case CONTELEC_SENSOR:
            contelec_service(position_feedback_ports.spi_ports, position_feedback_config.contelec_config, i_shared_memory, i_position_feedback);
            break;
        }
    }
}
