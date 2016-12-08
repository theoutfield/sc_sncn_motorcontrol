/*
 * position_feedback_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <position_feedback_service.h>
#include <print.h>
#include <xclib.h>

char start_message[] = ">>   SOMANET SENSOR SERVICE STARTING: ";

void fallback_service(PositionFeedbackConfig &position_feedback_config, server interface PositionFeedbackInterface i_position_feedback[3])
{
    printstr(start_message);
    printstrln("FALLBACK");

    timer t;
    unsigned ts;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (2000*20*250)) :> void;

    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        //receive config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                position_feedback_config = in_config;
                break;

        //send config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        //exit
        case i_position_feedback[int i].exit():
                loop_flag = 0;
                continue;

        case i_position_feedback[int i].set_position(int new_count):
                break;
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                break;
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                break;
        case i_position_feedback[int i].get_notification() -> int out_notification:
                break;
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                break;
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                break;
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                break;
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                break;
        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                break;
        }
    }
}

static void start_service(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports,
                   PositionFeedbackConfig &position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface i_position_feedback[3], static const int sensor_types)
{
    switch(position_feedback_config.sensor_type) {
//    case BISS_SENSOR:
//        if (sensor_types != 0)
//            biss_service(*qei_ports , position_feedback_config, i_shared_memory, i_position_feedback);
//        break;
    case CONTELEC_SENSOR:
        contelec_service(*spi_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case AMS_SENSOR:
        ams_service(*spi_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case HALL_SENSOR:
        hall_service(*hall_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case QEI_SENSOR:
        qei_service(*qei_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    default:
        fallback_service(position_feedback_config, i_position_feedback);
        break;
    }
}

static void start_service2(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports,
                   PositionFeedbackConfig &position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface i_position_feedback[3], static const int sensor_types)
{
    switch(position_feedback_config.sensor_type) {
    case BISS_SENSOR:
        if (sensor_types & 0b00100)
            biss_service(*qei_ports , position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case CONTELEC_SENSOR:
        contelec_service(*spi_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case AMS_SENSOR:
        ams_service(*spi_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case HALL_SENSOR:
        hall_service(*hall_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case QEI_SENSOR:
        qei_service(*qei_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    default:
        fallback_service(position_feedback_config, i_position_feedback);
        break;
    }
}

void check_ports(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports, PositionFeedbackConfig &position_feedback_config)
{
    //if ports are missing use fallback service
    if ( (position_feedback_config.sensor_type == HALL_SENSOR && hall_ports == null) ||
         ((position_feedback_config.sensor_type == BISS_SENSOR || position_feedback_config.sensor_type == QEI_SENSOR) && qei_ports == null) ||
         ((position_feedback_config.sensor_type == CONTELEC_SENSOR || position_feedback_config.sensor_type == AMS_SENSOR) && spi_ports == null) ) {
        position_feedback_config.sensor_type = 0;
    }
    if (position_feedback_config.sensor_type == BISS_SENSOR) {
        if (isnull((*qei_ports).p_qei_config))
                position_feedback_config.sensor_type = 0;
        if (spi_ports == null)
            position_feedback_config.sensor_type = 0;
    }
}

void reset_ports(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports)
{
    if (spi_ports != null) {
        set_clock_on((*spi_ports).spi_interface.blk2);
        set_clock_on((*spi_ports).spi_interface.blk1);
        set_port_use_on((*spi_ports).spi_interface.mosi);
        set_port_use_on((*spi_ports).spi_interface.miso);
        set_port_use_on((*spi_ports).spi_interface.sclk);
    }
    if (qei_ports != null) {
        set_port_use_on((*qei_ports).p_qei);
        if (!isnull((*qei_ports).p_qei_config))
            set_port_use_on((*qei_ports).p_qei_config);
    }
}

void set_clock_biss(QEIPorts * qei_ports, SPIPorts * spi_ports, PositionFeedbackConfig &position_feedback_config)
{
    if (qei_ports != null && spi_ports != null) {
        configure_clock_rate((*spi_ports).spi_interface.blk1, position_feedback_config.biss_config.clock_dividend, position_feedback_config.biss_config.clock_divisor); // a/b MHz
        start_clock((*spi_ports).spi_interface.blk1);
    }
}

int tickstobits(uint32_t ticks)
{
    return (31-clz(ticks));
}

void position_feedback_service(HallPorts &?hall_ports, QEIPorts &?qei_ports, SPIPorts &?spi_ports,
                               HallPorts &?hall_2_ports, QEIPorts &?qei_2_ports,
                               PositionFeedbackConfig &?position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface (&?i_position_feedback_1)[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3],
                               PositionFeedbackConfig &?position_feedback_config_3,
                               client interface shared_memory_interface ?i_shared_memory_3,
                               server interface PositionFeedbackInterface (&?i_position_feedback_3)[3],
                               PositionFeedbackConfig &?position_feedback_config_4,
                               client interface shared_memory_interface ?i_shared_memory_4,
                               server interface PositionFeedbackInterface (&?i_position_feedback_4)[3])
{
    //pointers to ports 1
    HallPorts * movable hall_ports_1 = &hall_ports;
    QEIPorts * movable qei_ports_1 = &qei_ports;
    SPIPorts * movable spi_ports_1 = &spi_ports;
    HallPorts * movable hall_2_ports_1 = &hall_2_ports;
    QEIPorts * movable qei_2_ports_1 = &qei_2_ports;

    //pointers to ports 2
    HallPorts * movable hall_ports_2;
    QEIPorts * movable qei_ports_2;
    SPIPorts * movable spi_ports_2;
    HallPorts * movable hall_2_ports_2;
    QEIPorts * movable qei_2_ports_2;

    int num_sensors = 2;
    if (hall_2_ports_1 != null) {
        num_sensors = 4;
    }


    while(1) {
        //reset clocks and ports
        reset_ports(hall_ports_1, qei_ports_1, spi_ports_1);

        //FIXME qei/hall mode port configuration
        if (qei_ports_1 != null) {
            if (!isnull((*qei_ports_1).p_qei_config))
                (*qei_ports_1).p_qei_config <: SET_PORT1_AS_HALL_PORT2_AS_QEI;
        }

        //check sensor 1
        if (!isnull(position_feedback_config_1)) {
            check_ports(hall_ports_1, qei_ports_1, spi_ports_1, position_feedback_config_1);

            //set biss clock if needed
            if (position_feedback_config_1.sensor_type == BISS_SENSOR) {
                configure_out_port((*qei_ports_1).p_qei_config, (*spi_ports_1).spi_interface.blk1, BISS_CLK_PORT_HIGH);
                configure_in_port((*qei_ports_1).p_qei, (*spi_ports_1).spi_interface.blk1);
                if (isnull(position_feedback_config_2)) {
                    set_clock_biss(qei_ports_1, spi_ports_1, position_feedback_config_1);
                } else {
                    if (position_feedback_config_2.sensor_type != CONTELEC_SENSOR && position_feedback_config_2.sensor_type != AMS_SENSOR) {
                        set_clock_biss(qei_ports_1, spi_ports_1, position_feedback_config_1);
                    }
                }
            }
        }
        //check sensor 2
        if (!isnull(position_feedback_config_2) && !isnull(position_feedback_config_1)) {
            //set biss clock if needed
            if (position_feedback_config_2.sensor_type == BISS_SENSOR) {
                if (!isnull((*qei_ports_1).p_qei_config)) {
                    configure_out_port((*qei_ports_1).p_qei_config, (*spi_ports_1).spi_interface.blk1, BISS_CLK_PORT_HIGH);
                    configure_in_port((*qei_ports_1).p_qei, (*spi_ports_1).spi_interface.blk1);
                } else {
                    position_feedback_config_2.sensor_type = 0;
                }
            }

            //move unused ports to sensor 2
            if (hall_ports_1 != null && position_feedback_config_1.sensor_type != HALL_SENSOR)
                hall_ports_2 = move(hall_ports_1);
            if (qei_ports_1 != null && position_feedback_config_1.sensor_type != BISS_SENSOR && position_feedback_config_1.sensor_type != QEI_SENSOR)
                qei_ports_2 = move(qei_ports_1);
            if (spi_ports_1 != null && position_feedback_config_1.sensor_type != CONTELEC_SENSOR &&  position_feedback_config_1.sensor_type != AMS_SENSOR)
                spi_ports_2 = move(spi_ports_1);
            //check ports
            check_ports(hall_ports_2, qei_ports_2, spi_ports_2, position_feedback_config_2);

            //set biss clock if needed
            if (position_feedback_config_2.sensor_type == BISS_SENSOR) {
                set_clock_biss(qei_ports_2, spi_ports_2, position_feedback_config_2);
            }
        }

        if (hall_2_ports_1 != null) {
            //start services
            par {
                {//sensor 1
                    if (!isnull(i_position_feedback_1) && !isnull(position_feedback_config_1)) {
                        start_service(hall_ports_1, qei_ports_1, spi_ports_1, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1, 0b00011);
                    }
                }
                {//sensor 2
                    if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                        start_service(hall_ports_2, qei_ports_2, spi_ports_2, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2, 0b00011);
                    }
                }
                {//sensor 3
                    if (!isnull(i_position_feedback_3) && !isnull(position_feedback_config_3)) {
                        start_service(hall_2_ports_1, null, null, position_feedback_config_3, i_shared_memory_3, i_position_feedback_3, 0b00011);
                    }
                }
                {//sensor 4
                    //                QEIPorts * movable qei_2_ports = &qei_2_ports;
                    if (!isnull(i_position_feedback_4) && !isnull(position_feedback_config_4)) {
                        start_service(null, qei_2_ports_1, null, position_feedback_config_4, i_shared_memory_4, i_position_feedback_4, 0b00011);
                    }
                }
            }
        }
        else {
            par {
                {//sensor 1
                    if (!isnull(i_position_feedback_1) && !isnull(position_feedback_config_1)) {
                        start_service2(hall_ports_1, qei_ports_1, spi_ports_1, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1, 0b11111);
                    }
                }
                {//sensor 2
                    if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                        start_service2(hall_ports_2, qei_ports_2, spi_ports_2, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2, 0b11111);
                    }
                }
            }
        }

        //move back ports
        if (hall_ports_2 != null)
            hall_ports_1 = move(hall_ports_2);
        if (qei_ports_2 != null)
            qei_ports_1 = move(qei_ports_2);
        if (spi_ports_2 != null)
            spi_ports_1 = move(spi_ports_2);
    }
}
