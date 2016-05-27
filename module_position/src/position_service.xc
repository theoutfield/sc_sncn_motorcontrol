/*
 * position_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <biss_service.h>
#include <contelec_service.h>
#include <position_service.h>

void position_service(PositionPorts &position_ports, PositionConfig &position_config,
                      client interface shared_memory_interface ?i_shared_memory,
                      server interface PositionInterface i_position[3])
{
    for (int i=0; i<2; i++) {
        switch(position_config.sensor_type[i]) {
        case BISS_SENSOR:
            biss_service(position_ports.biss_ports, position_config.biss_config, i_shared_memory, i_position);
            break;
        case CONTELEC_SENSOR:
            contelec_service(position_ports.spi_ports, position_config.contelec_config, i_shared_memory, i_position);
            break;
        }
    }
}
