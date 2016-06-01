/*
 * position_feedback_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <position_feedback_service.h>


void position_feedback_service(PositionFeedbackPorts &?position_feedback_ports_1, PositionFeedbackConfig &?position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface ?i_position_feedback_1[3],
                               PositionFeedbackPorts &?position_feedback_ports_2, PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface ?i_position_feedback_2[3])
{

    par {
        //sensor 1
        {
            if (!isnull(i_position_feedback_1)) {
                switch(position_feedback_config_1.sensor_type) {
                case BISS_SENSOR:
                    biss_service(position_feedback_ports_1 , position_feedback_config_1.biss_config, i_shared_memory_1, i_position_feedback_1);
                    break;
                case CONTELEC_SENSOR:
                    contelec_service(position_feedback_ports_1, position_feedback_config_1.contelec_config, i_shared_memory_1, i_position_feedback_1);
                    break;
                }
            }
        }
        //sensor 2
        {
            if (!isnull(position_feedback_ports_2) && !isnull(position_feedback_config_2)) {
                switch(position_feedback_config_2.sensor_type) {
                case BISS_SENSOR:
                    biss_service(position_feedback_ports_2 , position_feedback_config_2.biss_config, i_shared_memory_2, i_position_feedback_2);
                    break;
                case CONTELEC_SENSOR:
                    contelec_service(position_feedback_ports_2, position_feedback_config_2.contelec_config, i_shared_memory_2, i_position_feedback_2);
                    break;
                }
            }
        }
    }
}
