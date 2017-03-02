/**
 * @file position_feedback_service.xc
 * @author Synapticon GmbH <support@synapticon.com>
 */


#include <position_feedback_service.h>
#include <serial_encoder_service.h>
#include <refclk.h>
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
                UsecType ifm_usec = position_feedback_config.ifm_usec;
                position_feedback_config = in_config;
                position_feedback_config.ifm_usec = ifm_usec;
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
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                break;
        case i_position_feedback[int i].get_notification() -> int out_notification:
                break;
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                break;
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position, unsigned int status }:
                break;
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                break;

        //gpio
        case t when timerafter(ts + (1000*position_feedback_config.ifm_usec)) :> ts:
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory);
                break;
        }
    }
}

void start_service(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4],
                   int hall_enc_select_config,
                   PositionFeedbackConfig &position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface i_position_feedback[3])
{
    switch(position_feedback_config.sensor_type) {
    case BISS_SENSOR:
    case REM_16MT_SENSOR:
    case REM_14_SENSOR:
        serial_encoder_service(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports, hall_enc_select_config, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    case HALL_SENSOR:
        if (position_feedback_config.hall_config.port_number == ENCODER_PORT_1) {
            hall_service(*qei_hall_port_1, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        } else if (position_feedback_config.hall_config.port_number == ENCODER_PORT_2) {
            hall_service(*qei_hall_port_2, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        }
        break;
    case QEI_SENSOR:
        if (position_feedback_config.qei_config.port_number == ENCODER_PORT_1) {
            qei_service(*qei_hall_port_1, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        } else if (position_feedback_config.qei_config.port_number == ENCODER_PORT_2) {
            qei_service(*qei_hall_port_2, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        }
        break;
    default:
        fallback_service(gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback);
        break;
    }
}

void check_ports(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4], int gpio_ports_check,
        int &hall_enc_select_config, PositionFeedbackConfig &position_feedback_config, PositionFeedbackConfig &?position_feedback_config_2)
{
    //if ports are missing use fallback service
    if (position_feedback_config.sensor_type == HALL_SENSOR)
    {
        if ( (position_feedback_config.hall_config.port_number == ENCODER_PORT_1 && qei_hall_port_1 == null) ||
             (position_feedback_config.hall_config.port_number == ENCODER_PORT_2 && qei_hall_port_2 == null) )
        {
            position_feedback_config.sensor_type = 0;
        }
        hall_enc_select_config &= ~(1 << position_feedback_config.hall_config.port_number) ; //TTL mode
    }
    else if (position_feedback_config.sensor_type == QEI_SENSOR)
    {
        if ( (position_feedback_config.qei_config.port_number == ENCODER_PORT_1 && qei_hall_port_1 == null) ||
             (position_feedback_config.qei_config.port_number == ENCODER_PORT_2 && qei_hall_port_2 == null) )
        {
            position_feedback_config.sensor_type = 0;
        }
        //set qei_hall_port to TTL or RS422 (differential) mode
        if (position_feedback_config.qei_config.signal_type == QEI_TTL_SIGNAL) {
            hall_enc_select_config &= ~(1 << position_feedback_config.qei_config.port_number) ; //TTL mode
        } else {
            hall_enc_select_config |=  (1 << position_feedback_config.qei_config.port_number);  //RS422 (differential) mode
        }
    }
    else if ((position_feedback_config.sensor_type == REM_16MT_SENSOR || position_feedback_config.sensor_type == REM_14_SENSOR)) {
        if (spi_ports == null || gpio_ports_check == 0 || //check if we have all needed ports
            (!isnull(position_feedback_config_2) && ((position_feedback_config_2.sensor_type == BISS_SENSOR && position_feedback_config_2.biss_config.clock_port_config <= BISS_CLOCK_PORT_EXT_D3)
            || position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR)) // or biss with gpio port for clock
        ) {
            position_feedback_config.sensor_type = 0;
        }
    }
    else if (position_feedback_config.sensor_type == BISS_SENSOR) {
        if (spi_ports == null) {
            position_feedback_config.sensor_type = 0;
        } else {
            //check and configure data port
            if (position_feedback_config.biss_config.data_port_number == ENCODER_PORT_1) {
                if (qei_hall_port_1 == null) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_in_port(qei_hall_port_1->p_qei_hall, (*spi_ports).spi_interface.blk1);
                }
            } else if (position_feedback_config.biss_config.data_port_number == ENCODER_PORT_2) {
                if (qei_hall_port_2 == null) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_in_port(qei_hall_port_2->p_qei_hall, (*spi_ports).spi_interface.blk1);
                }
            }
            hall_enc_select_config |=  (1 << position_feedback_config.biss_config.data_port_number);  //RS422 (differential) mode
            //check and configure clock port
            if (position_feedback_config.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4) { //hall_enc_select_port clock port
                if (hall_enc_select_port == null) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_out_port(hall_enc_select_port->p_hall_enc_select, (*spi_ports).spi_interface.blk1, hall_enc_select_config);
                }
            } else { //gpio clock port
                if (gpio_ports[position_feedback_config.biss_config.clock_port_config] == null ||
                    (!isnull(position_feedback_config_2) && (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR) ) )
                {
                    position_feedback_config.sensor_type = 0;
                } else {
                    set_port_use_on(*gpio_ports[position_feedback_config.biss_config.clock_port_config]);
                    configure_out_port(*gpio_ports[position_feedback_config.biss_config.clock_port_config], (*spi_ports).spi_interface.blk1, 1);
                }
            }
            //configure clock rate
            set_clock_on((*spi_ports).spi_interface.blk1);
            configure_clock_rate_at_most((*spi_ports).spi_interface.blk1, position_feedback_config.ifm_usec, ((position_feedback_config.ifm_usec*1000)/(position_feedback_config.biss_config.clock_frequency*2))+1);
            start_clock((*spi_ports).spi_interface.blk1);
        }
    }
    if (hall_enc_select_port != null) {
        if (!isnull(hall_enc_select_port->p_hall_enc_select)) {
            hall_enc_select_port->p_hall_enc_select <: hall_enc_select_config;
        }
    }

}

void reset_ports(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4])
{
    if (spi_ports != null) {
        set_clock_on((*spi_ports).spi_interface.blk2);
        set_clock_on((*spi_ports).spi_interface.blk1);
    }
#if 0
        if ((*spi_ports).spi_interface.sclk != null) {
            set_port_use_on(*(*spi_ports).spi_interface.mosi);
            set_port_use_on(*(*spi_ports).spi_interface.miso);
            set_port_use_on(*(*spi_ports).spi_interface.sclk);
        }
    }
#endif

    for (int i=0 ; i<4 ; i++) {
        if (gpio_ports[i] != null) {
            set_port_use_on(*gpio_ports[i]);
        }
    }

    if (qei_hall_port_1 != null) {
        set_port_use_on(qei_hall_port_1->p_qei_hall);
    }
    if (qei_hall_port_2 != null) {
        set_port_use_on(qei_hall_port_2->p_qei_hall);
    }
    if (hall_enc_select_port != null) {
        if (!isnull((*hall_enc_select_port).p_hall_enc_select))
            set_port_use_on((*hall_enc_select_port).p_hall_enc_select);
    }
}

int tickstobits(uint32_t ticks)
{
    return (31-clz(ticks));
}

void multiturn(int &count, int last_position, int position, int ticks_per_turn) {
        int difference = position - last_position;
        if (difference >= ticks_per_turn/2)
            count = count + difference - ticks_per_turn;
        else if (-difference >= ticks_per_turn/2)
            count = count + difference + ticks_per_turn;
        else
            count += difference;
}


int velocity_compute(int difference, int timediff, int resolution)
{
    return (difference * (60000000/timediff)) / resolution;
}

void write_shared_memory(client interface shared_memory_interface ?i_shared_memory, SensorFunction sensor_function, int count, int velocity, int angle, int hall_state)
{
    if (!isnull(i_shared_memory)) {
        switch(sensor_function)
        {
        case SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL:
            i_shared_memory.write_angle_and_primary_feedback(angle, hall_state, count, velocity);
            break;
        case SENSOR_FUNCTION_COMMUTATION_AND_FEEDBACK_ONLY:
            i_shared_memory.write_angle_and_secondary_feedback(angle, hall_state, count, velocity);
            break;
        case SENSOR_FUNCTION_MOTION_CONTROL:
            i_shared_memory.write_primary_feedback(count, velocity);
            break;
        case SENSOR_FUNCTION_FEEDBACK_ONLY:
            i_shared_memory.write_secondary_feedback(count, velocity);
            break;
        }
    }
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

void position_feedback_service(QEIHallPort &?qei_hall_port_1, QEIHallPort &?qei_hall_port_2, HallEncSelectPort &?hall_enc_select_port, SPIPorts &?spi_ports, port ?gpio_port_0, port ?gpio_port_1, port ?gpio_port_2, port ?gpio_port_3,
                               PositionFeedbackConfig &position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3])
{
    if (position_feedback_config_1.ifm_usec == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    //pointers to ports 1
    QEIHallPort * movable qei_hall_port_1_1 = &qei_hall_port_1;
    QEIHallPort * movable qei_hall_port_2_1 = &qei_hall_port_2;
    HallEncSelectPort * movable hall_enc_select_port_1 = &hall_enc_select_port;
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
            gpio_ports[0] = reconfigure_port(move(gpio_0), port);
            gpio_ports[1] = reconfigure_port(move(gpio_1), port);
            gpio_ports[2] = reconfigure_port(move(gpio_2), port);
            gpio_ports[3] = reconfigure_port(move(gpio_3), port);
    }

    //pointers to ports 2
    QEIHallPort * movable qei_hall_port_1_2;
    QEIHallPort * movable qei_hall_port_2_2;
    HallEncSelectPort * movable hall_enc_select_port_2;
    SPIPorts * movable spi_ports_2;
    port * movable gpio_ports_2[4];

    int spi_on = 0;
    int hall_enc_select_config = 0b0011; // set qei_hall_port_1 and 2 to RS422 (differential) mode

    while(1) {
        //reset clocks and ports
        reset_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port_1, spi_ports_1, gpio_ports);

        //config gpio ports
        for (int i=0 ; i<4 ; i++) {
            if (gpio_ports[i] != null) {
                if (position_feedback_config_1.gpio_config[i] == GPIO_INPUT_PULLDOWN) {
                    set_port_pull_down(*gpio_ports[i]);
                } else {
                    set_port_pull_none(*gpio_ports[i]);
                }
            }
        }

        //checks ports sensor 2
        if (!isnull(position_feedback_config_2)) {
            //check an configure ports, set to fallback service if incorrect configuration
            check_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port_1, spi_ports_1, gpio_ports, gpio_ports_check, hall_enc_select_config, position_feedback_config_2, position_feedback_config_1);

            //move all needed ports to ports pointers number 2
            switch(position_feedback_config_2.sensor_type) {
            case HALL_SENSOR:
                if (position_feedback_config_2.hall_config.port_number == ENCODER_PORT_1) {
                    qei_hall_port_1_2 = move(qei_hall_port_1_1);
                } else if (position_feedback_config_2.hall_config.port_number == ENCODER_PORT_2) {
                    qei_hall_port_2_2 = move(qei_hall_port_2_1);
                }
                break;
            case QEI_SENSOR:
                if (position_feedback_config_2.qei_config.port_number == ENCODER_PORT_1) {
                    qei_hall_port_1_2 = move(qei_hall_port_1_1);
                } else if (position_feedback_config_2.qei_config.port_number == ENCODER_PORT_2) {
                    qei_hall_port_2_2 = move(qei_hall_port_2_1);
                }
                break;
            case BISS_SENSOR:
                //move data port
                if (position_feedback_config_2.biss_config.data_port_number == ENCODER_PORT_1) {
                    qei_hall_port_1_2 = move(qei_hall_port_1_1);
                } else if (position_feedback_config_2.biss_config.data_port_number == ENCODER_PORT_2) {
                    qei_hall_port_2_2 = move(qei_hall_port_2_1);
                }
                //move clock port
                if (position_feedback_config_2.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4) { //hall_enc_select_port clock port
                    hall_enc_select_port_2 = move(hall_enc_select_port_1);
                } else { //gpio clock port
                    gpio_ports_2[position_feedback_config_2.biss_config.clock_port_config] = move(gpio_ports[position_feedback_config_2.biss_config.clock_port_config]);
                }
                break;
            }
        }

        //checks ports sensor 1
        //check an configure ports, set to fallback service if incorrect configuration
        check_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port_1, spi_ports_1, gpio_ports, gpio_ports_check, hall_enc_select_config, position_feedback_config_1, position_feedback_config_2);

        //additional config for spi services
        //move gpio ports to spi ports if needed, move spi ports to service 2 if needed
        if ( position_feedback_config_1.sensor_type == REM_16MT_SENSOR || position_feedback_config_1.sensor_type == REM_14_SENSOR)
        {
            set_clock_on((*spi_ports_1).spi_interface.blk1);
            (*spi_ports_1).slave_select = reconfigure_port(move(gpio_ports[0]), port);
            (*spi_ports_1).spi_interface.sclk = reconfigure_port(move(gpio_ports[1]), out buffered port:8);
            (*spi_ports_1).spi_interface.miso = reconfigure_port(move(gpio_ports[2]), in buffered port:8);
            (*spi_ports_1).spi_interface.mosi = reconfigure_port(move(gpio_ports[3]), out buffered port:8);
            spi_on = 1;
        }
        if (!isnull(position_feedback_config_2)) {
            if (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR) {
                set_clock_on((*spi_ports_1).spi_interface.blk1);
                (*spi_ports_1).slave_select = reconfigure_port(move(gpio_ports[0]), port);
                (*spi_ports_1).spi_interface.sclk = reconfigure_port(move(gpio_ports[1]), out buffered port:8);
                (*spi_ports_1).spi_interface.miso = reconfigure_port(move(gpio_ports[2]), in buffered port:8);
                (*spi_ports_1).spi_interface.mosi = reconfigure_port(move(gpio_ports[3]), out buffered port:8);
                spi_ports_2 = move(spi_ports_1);
                spi_on = 1;
            }
        }

        //start services
        par {
            {//sensor 1
                start_service(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port_1, spi_ports_1, gpio_ports, hall_enc_select_config, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1);
            }
            {//sensor 2
                if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                    start_service(qei_hall_port_1_2, qei_hall_port_2_2, hall_enc_select_port_2, spi_ports_2, gpio_ports_2, hall_enc_select_config, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2);
                }
            }
        }

        //move back ports
        if (qei_hall_port_1_2 != null)
            qei_hall_port_1_1 = move(qei_hall_port_1_2);
        if (qei_hall_port_2_2 != null)
            qei_hall_port_2_1 = move(qei_hall_port_2_2);
        if (hall_enc_select_port_2 != null)
            hall_enc_select_port_1 = move(hall_enc_select_port_2);

        for (int i=0 ; i<4 ; i++) {
            if (gpio_ports_2[i] != null)
                gpio_ports[i] = move(gpio_ports_2[i]);
        }

        if (spi_ports_2 != null) {
            spi_ports_1 = move(spi_ports_2);
        }
        if (spi_on) {
            gpio_ports[0] = reconfigure_port(move((*spi_ports_1).slave_select), port);
            gpio_ports[1] = reconfigure_port(move((*spi_ports_1).spi_interface.sclk), port);
            gpio_ports[2] = reconfigure_port(move((*spi_ports_1).spi_interface.miso), port);
            gpio_ports[3] = reconfigure_port(move((*spi_ports_1).spi_interface.mosi), port);
            spi_on = 0;
        }
    }
}
