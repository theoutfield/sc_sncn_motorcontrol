/*
 * position_feedback_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <position_feedback_service.h>
#include <print.h>

void start_service(HallPorts &?hall_ports, BISSPorts &?biss_ports, SPIPorts &?spi_ports,
                   PositionFeedbackConfig &?position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface ?i_position_feedback[3])
{
    switch(position_feedback_config.sensor_type) {
    case BISS_SENSOR:
        biss_service(biss_ports , position_feedback_config.biss_config, i_shared_memory, i_position_feedback);
        break;
    case CONTELEC_SENSOR:
        contelec_service(spi_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case HALL_SENSOR:
        hall_service(hall_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case QEI_SENSOR:
        qei_service(biss_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    }
}


void position_feedback_service(HallPorts &?hall_ports, BISSPorts &?biss_ports, SPIPorts &?spi_ports,
                               PositionFeedbackConfig &?position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface ?i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface ?i_position_feedback_2[3])
{
    //pointers to ports 1
    HallPorts * movable hall_ports_1 = &hall_ports;
    BISSPorts * movable biss_ports_1 = &biss_ports;
    SPIPorts * movable spi_ports_1 = &spi_ports;

    //pointers to ports 2
    HallPorts * movable hall_ports_2;
    BISSPorts * movable biss_ports_2;
    SPIPorts * movable spi_ports_2;

    if (biss_ports_1 == null) {
        printstrln("biss_ports_1 is null");
    } else {
        printstrln("biss_ports_1 is not null");
    }

    (*biss_ports_1).p_biss_clk <: SET_PORT1_AS_HALL_PORT2_AS_QEI;

//    spi_ports_1 = move(spi_ports_0);
//    hall_ports_1 = move(hall_ports_0);
//    biss_ports_1 = move(biss_ports_0);



    while(1) {
        //set ports





        par {
            //sensor 1
            {
                if (!isnull(i_position_feedback_1) && !isnull(position_feedback_config_1)) {
                    start_service(*hall_ports_1, *biss_ports_1, *spi_ports_1, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1);
                }
            }
            //sensor 2
            {
                if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                    start_service(*hall_ports_2, *biss_ports_2, *spi_ports_2, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2);
                }
            }
        }
    }
}
