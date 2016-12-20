/*
 * position_feedback_service.xc
 *
 *  Created on: May 27, 2016
 *      Author: romuald
 */

#include <position_feedback_service.h>
#include <serial_encoder_service.h>
#include <print.h>
#include <xclib.h>
#include <xs1.h>

#ifdef DEBUG_POSITION_FEEDBACK
char start_message[] = ">>   SOMANET SENSOR SERVICE STARTING: ";
#endif

void fallback_service(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config,
                      client interface shared_memory_interface ?i_shared_memory,
                      server interface PositionFeedbackInterface i_position_feedback[3])
{
#ifdef DEBUG_POSITION_FEEDBACK
    printstr(start_message);
    printstrln("FALLBACK");
#endif

    timer t;
    unsigned ts;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (2000*20*250)) :> ts;

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

        //gpio read
        case i_position_feedback[int i].gpio_read(int gpio_number) -> int out_value:
                out_value = gpio_read(gpio_ports, position_feedback_config, gpio_number);
                break;

        //gpio_write
        case i_position_feedback[int i].gpio_write(int gpio_number, int in_value):
                gpio_write(gpio_ports, position_feedback_config, gpio_number, in_value);
                break;

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

        //gpio
        case t when timerafter(ts + (1000*IFM_TILE_USEC)) :> ts:
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory);
                break;
        }
    }
}

void start_service(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports, port * (&?gpio_ports)[4],
                   PositionFeedbackConfig &position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface i_position_feedback[3])
{
    switch(position_feedback_config.sensor_type) {
    case BISS_SENSOR:
    case REM_16MT_SENSOR:
    case REM_14_SENSOR:
        serial_encoder_service(spi_ports, qei_ports, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case HALL_SENSOR:
        hall_service(*hall_ports, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case QEI_SENSOR:
        qei_service(*qei_ports, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    default:
        fallback_service(gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    }
}

void check_ports(HallPorts * hall_ports, QEIPorts * qei_ports, SPIPorts * spi_ports, PositionFeedbackConfig &position_feedback_config)
{
    //if ports are missing use fallback service
    if ( (position_feedback_config.sensor_type == HALL_SENSOR && hall_ports == null) ||
         ((position_feedback_config.sensor_type == BISS_SENSOR || position_feedback_config.sensor_type == QEI_SENSOR) && qei_ports == null) )
    {
        position_feedback_config.sensor_type = 0;
    }
    if ((position_feedback_config.sensor_type == REM_16MT_SENSOR || position_feedback_config.sensor_type == REM_14_SENSOR)) {
        if (spi_ports == null) {
            position_feedback_config.sensor_type = 0;
        } else if ((*spi_ports).slave_select == null) {
            position_feedback_config.sensor_type = 0;
        }
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
        if ((*spi_ports).spi_interface.sclk != null) {
            set_port_use_on(*(*spi_ports).spi_interface.mosi);
            set_port_use_on(*(*spi_ports).spi_interface.miso);
            set_port_use_on(*(*spi_ports).spi_interface.sclk);
        }
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

int gpio_read(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number)
{
    int out_value = -1;
    if (!isnull(gpio_ports) && gpio_number >= 0 && gpio_number < 4 ) {
        if ( gpio_ports[gpio_number] != null &&
                (position_feedback_config.gpio_config[gpio_number] == GPIO_INPUT || position_feedback_config.gpio_config[gpio_number] == GPIO_INPUT_PULLDOWN) )
        {
            *gpio_ports[gpio_number] :> out_value;
        }
    }
    return out_value;
}

void gpio_write(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, int gpio_number, int value)
{
    if (!isnull(gpio_ports) && gpio_number >= 0 && gpio_number < 4 ) {
        if ( gpio_ports[gpio_number] != null && position_feedback_config.gpio_config[gpio_number] == GPIO_OUTPUT)
        {
            *gpio_ports[gpio_number] <: value;
        }
    }
}

void gpio_shared_memory(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory)
{
    if (!isnull(gpio_ports) && !isnull(i_shared_memory)) {
        unsigned int gpio_input = 0;
        for (int i=0 ; i<4 ; i++) {
            if ( gpio_ports[i] != null &&
                    (position_feedback_config.gpio_config[i] == GPIO_INPUT || position_feedback_config.gpio_config[i] == GPIO_INPUT_PULLDOWN) )
            {
                unsigned int bit;
                *gpio_ports[i] :> bit;
                gpio_input += (bit&1)<<i;
            }
        }
        unsigned int gpio_output = i_shared_memory.gpio_write_input_read_output(gpio_input);
        for (int i=0 ; i<4 ; i++) {
            if ( gpio_ports[i] != null && position_feedback_config.gpio_config[i] == GPIO_OUTPUT)
            {
                *gpio_ports[i] <: (gpio_output>>i)&1;
            }
        }
    }
}

typedef struct {
    port a;
} PORTS;

typedef struct {
    port * movable a;
} PORTS2;

void position_feedback_service(HallPorts &?hall_ports, QEIPorts &?qei_ports, SPIPorts &?spi_ports, port ?gpio_port_0, port ?gpio_port_1, port ?gpio_port_2, port ?gpio_port_3,
                               PositionFeedbackConfig &position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3])
{
    //pointers to ports 1
    HallPorts * movable hall_ports_1 = &hall_ports;
    QEIPorts * movable qei_ports_1 = &qei_ports;
    SPIPorts * movable spi_ports_1 = &spi_ports;

    //gpio ports
    int gpio_ports_check = 1;
    if (isnull(gpio_port_0) || isnull(gpio_port_1) || isnull(gpio_port_2) || isnull(gpio_port_3))
        gpio_ports_check = 0;
    port? * movable gpio_0 = &gpio_port_0;
    port? * movable gpio_1 = &gpio_port_1;
    port? * movable gpio_2 = &gpio_port_2;
    port? * movable gpio_3 = &gpio_port_3;
    port * movable gpio_ports[4];
    if (gpio_ports_check) {
        if (spi_ports_1 != null) {
            (*spi_ports_1).slave_select = reconfigure_port(move(gpio_0), port);
            (*spi_ports_1).spi_interface.sclk = reconfigure_port(move(gpio_1), out buffered port:8);
            (*spi_ports_1).spi_interface.miso = reconfigure_port(move(gpio_2), in buffered port:8);
            (*spi_ports_1).spi_interface.mosi = reconfigure_port(move(gpio_3), out buffered port:8);
        } else {
            gpio_ports[0] = reconfigure_port(move(gpio_0), port);
            gpio_ports[1] = reconfigure_port(move(gpio_1), port);
            gpio_ports[2] = reconfigure_port(move(gpio_2), port);
            gpio_ports[3] = reconfigure_port(move(gpio_3), port);
        }
    }

    //pointers to ports 2
    HallPorts * movable hall_ports_2;
    QEIPorts * movable qei_ports_2;
    SPIPorts * movable spi_ports_2;


    while(1) {
        //reset clocks and ports
        reset_ports(hall_ports_1, qei_ports_1, spi_ports_1);

        //FIXME qei/hall mode port configuration
        if (qei_ports_1 != null) {
            if (!isnull((*qei_ports_1).p_qei_config))
                (*qei_ports_1).p_qei_config <: SET_PORT1_AS_HALL_PORT2_AS_QEI;
        }

        //check sensor 1
        check_ports(hall_ports_1, qei_ports_1, spi_ports_1, position_feedback_config_1);

        //set biss clock if needed
        if (position_feedback_config_1.sensor_type == BISS_SENSOR) {
            configure_out_port((*qei_ports_1).p_qei_config, (*spi_ports_1).spi_interface.blk1, BISS_CLK_PORT_HIGH);
            configure_in_port((*qei_ports_1).p_qei, (*spi_ports_1).spi_interface.blk1);
            if (isnull(position_feedback_config_2)) {
                set_clock_biss(qei_ports_1, spi_ports_1, position_feedback_config_1);
            } else {
                if (position_feedback_config_2.sensor_type != REM_16MT_SENSOR && position_feedback_config_2.sensor_type != REM_14_SENSOR) {
                    set_clock_biss(qei_ports_1, spi_ports_1, position_feedback_config_1);
                }
            }
        }
        int move_gpio_check = 1;
        //check sensor 2
        if (!isnull(position_feedback_config_2)) {
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
            if (spi_ports_1 != null && position_feedback_config_1.sensor_type != REM_16MT_SENSOR && position_feedback_config_1.sensor_type != REM_14_SENSOR)
                spi_ports_2 = move(spi_ports_1);
            //check ports
            check_ports(hall_ports_2, qei_ports_2, spi_ports_2, position_feedback_config_2);

            //set biss clock if needed
            if (position_feedback_config_2.sensor_type == BISS_SENSOR) {
                set_clock_biss(qei_ports_2, spi_ports_2, position_feedback_config_2);
            }

            if (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR) {
                move_gpio_check = 0;
            }
        }

        //move gpio ports if no sensor is using spi
        if (position_feedback_config_1.sensor_type != REM_16MT_SENSOR && position_feedback_config_1.sensor_type != REM_14_SENSOR && gpio_ports_check && move_gpio_check)
        {
            if (spi_ports_2 != null) {
                gpio_ports[0] = reconfigure_port(move((*spi_ports_2).slave_select), port);
                gpio_ports[1] = reconfigure_port(move((*spi_ports_2).spi_interface.sclk), port);
                gpio_ports[2] = reconfigure_port(move((*spi_ports_2).spi_interface.miso), port);
                gpio_ports[3] = reconfigure_port(move((*spi_ports_2).spi_interface.mosi), port);
            }
            for (int i=0 ; i<4 ; i++) {
                if (position_feedback_config_1.gpio_config[i] == GPIO_INPUT_PULLDOWN) {
                    set_port_pull_down(*gpio_ports[i]);
                } else {
                    set_port_pull_none(*gpio_ports[i]);
                }
            }
        }


        //start services
        par {
            {//sensor 1
                start_service(hall_ports_1, qei_ports_1, spi_ports_1, gpio_ports, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1);
            }
            {//sensor 2
                if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                    start_service(hall_ports_2, qei_ports_2, spi_ports_2, null, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2);
                }
            }
        }

        //move back ports
        if (hall_ports_2 != null)
            hall_ports_1 = move(hall_ports_2);
        if (qei_ports_2 != null)
            qei_ports_1 = move(qei_ports_2);
        if (spi_ports_2 != null) {
            spi_ports_1 = move(spi_ports_2);
            if (gpio_ports_check) {
                (*spi_ports_1).slave_select = reconfigure_port(move(gpio_ports[0]), port);
                (*spi_ports_1).spi_interface.sclk = reconfigure_port(move(gpio_ports[1]), out buffered port:8);
                (*spi_ports_1).spi_interface.miso = reconfigure_port(move(gpio_ports[2]), in buffered port:8);
                (*spi_ports_1).spi_interface.mosi = reconfigure_port(move(gpio_ports[3]), out buffered port:8);
            }
        }
    }
}
