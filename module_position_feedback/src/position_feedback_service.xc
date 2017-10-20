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
                      server interface PositionFeedbackInterface i_position_feedback[3],
                      int gpio_on)
{
#ifdef DEBUG_POSITION_FEEDBACK
    printstr(start_message);
    printstrln("FALLBACK");
#endif

    timer t;
    unsigned ts;

    ////proper task startup
    //t :> ts;
    //t when timerafter (ts + (2000*20*250)) :> ts;

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
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position, SensorError status }:
                break;
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                break;

        //gpio
        case t when timerafter(ts + (1000*position_feedback_config.ifm_usec)) :> ts:
                gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory, gpio_on);
                break;
        }
    }
}

void start_service(port * qei_hall_port_1, port * qei_hall_port_2, port * biss_clock_port, port * biss_data_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4],
                   int biss_clock_low, int biss_clock_high,
                   PositionFeedbackConfig &position_feedback_config,
                   client interface shared_memory_interface ?i_shared_memory,
                   server interface PositionFeedbackInterface i_position_feedback[3],
                   int gpio_on)
{
    switch(position_feedback_config.sensor_type) {
    case BISS_SENSOR:
    case SSI_SENSOR:
    case REM_16MT_SENSOR:
    case REM_14_SENSOR:
        serial_encoder_service(biss_clock_port, biss_data_port, spi_ports, gpio_ports, biss_clock_low, biss_clock_high, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        break;
    case HALL_SENSOR:
        if (position_feedback_config.hall_config.port_number == ENCODER_PORT_1) {
            hall_service(*qei_hall_port_1, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        } else if (position_feedback_config.hall_config.port_number == ENCODER_PORT_2) {
            hall_service(*qei_hall_port_2, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        }
        break;
    case QEI_SENSOR:
        if (position_feedback_config.qei_config.port_number == ENCODER_PORT_1) {
            qei_service(*qei_hall_port_1, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        } else if (position_feedback_config.qei_config.port_number == ENCODER_PORT_2) {
            qei_service(*qei_hall_port_2, gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        }
        break;
    default:
        fallback_service(gpio_ports, position_feedback_config, i_shared_memory, i_position_feedback, gpio_on);
        break;
    }
}

void set_hall_enc_select_config(int &hall_enc_select_config, EncoderPortNumber port_number, EncoderPortSignalType signal_type)
{
    int mask = 0b0001; //port 1
    if (port_number == ENCODER_PORT_2) {
        mask = 0b0010; //port 2
    }
    if (signal_type == ENCODER_PORT_TTL_SIGNAL) {
        hall_enc_select_config &= ~mask ; //TTL mode
    } else {
        hall_enc_select_config |=  mask;  //RS422 (differential) mode
    }
}

void check_ports(port * qei_hall_port_1, port * qei_hall_port_2, port * hall_enc_select_port, SPIPorts * spi_ports, port * gpio_ports[4], PositionFeedbackPortsCheck ports_check,
        int &hall_enc_select_config, unsigned int hall_enc_select_port_inv_mask, PositionFeedbackConfig &position_feedback_config, PositionFeedbackConfig &?position_feedback_config_2)
{
    //if ports are missing use fallback service
    if (position_feedback_config.sensor_type == HALL_SENSOR)
    {
        if ( (position_feedback_config.hall_config.port_number == ENCODER_PORT_1 && ports_check.qei_hall_port_1 != POSITION_FEEDBACK_PORTS_1) ||
             (position_feedback_config.hall_config.port_number == ENCODER_PORT_2 && ports_check.qei_hall_port_2 != POSITION_FEEDBACK_PORTS_1) )
        {
            position_feedback_config.sensor_type = 0;
        }
        set_hall_enc_select_config(hall_enc_select_config, position_feedback_config.hall_config.port_number, ENCODER_PORT_TTL_SIGNAL);
    }
    else if (position_feedback_config.sensor_type == QEI_SENSOR)
    {
        if ( (position_feedback_config.qei_config.port_number == ENCODER_PORT_1 && ports_check.qei_hall_port_1 != POSITION_FEEDBACK_PORTS_1) ||
             (position_feedback_config.qei_config.port_number == ENCODER_PORT_2 && ports_check.qei_hall_port_2 != POSITION_FEEDBACK_PORTS_1) )
        {
            position_feedback_config.sensor_type = 0;
        }
        //set qei_hall_port to TTL or RS422 (differential) mode
        set_hall_enc_select_config(hall_enc_select_config, position_feedback_config.qei_config.port_number, position_feedback_config.qei_config.signal_type);
    }
    else if ((position_feedback_config.sensor_type == REM_16MT_SENSOR || position_feedback_config.sensor_type == REM_14_SENSOR)) {
        if (ports_check.spi_ports != POSITION_FEEDBACK_PORTS_1 ||
            ports_check.gpio_ports[0] != POSITION_FEEDBACK_PORTS_1 || ports_check.gpio_ports[1] != POSITION_FEEDBACK_PORTS_1 ||
            ports_check.gpio_ports[2] != POSITION_FEEDBACK_PORTS_1 || ports_check.gpio_ports[3] != POSITION_FEEDBACK_PORTS_1 || //check if we have all needed ports
            (!isnull(position_feedback_config_2) && ((position_feedback_config_2.sensor_type == BISS_SENSOR && position_feedback_config_2.biss_config.clock_port_config <= BISS_CLOCK_PORT_EXT_D3)
            || position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR)) // or biss with gpio port for clock
        ) {
            position_feedback_config.sensor_type = 0;
        }
    }
    else if (position_feedback_config.sensor_type == BISS_SENSOR || position_feedback_config.sensor_type == SSI_SENSOR) {
        if (ports_check.spi_ports != POSITION_FEEDBACK_PORTS_1) {
            position_feedback_config.sensor_type = 0;
        } else {
            //check and configure data port
            if (position_feedback_config.biss_config.data_port_number == ENCODER_PORT_1) {
                if (ports_check.qei_hall_port_1 != POSITION_FEEDBACK_PORTS_1) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_in_port(*qei_hall_port_1, spi_ports->spi_interface.blk1);
                    //if the clock port is GPIO 3, enable the redirect GPIO 3 -> qei_hall_port_1
                    //by setting P4F2 to high (3rd bit of hall_enc_select_config)
                    if (position_feedback_config.biss_config.clock_port_config == BISS_CLOCK_PORT_EXT_D3) {
                        hall_enc_select_config |= 0b0100;
                    }
                }
            } else if (position_feedback_config.biss_config.data_port_number == ENCODER_PORT_2) {
                if (ports_check.qei_hall_port_2 != POSITION_FEEDBACK_PORTS_1) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_in_port(*qei_hall_port_2, spi_ports->spi_interface.blk1);
                }
            } else if (position_feedback_config.biss_config.data_port_number >= ENCODER_PORT_EXT_D0 && position_feedback_config.biss_config.data_port_number <= ENCODER_PORT_EXT_D3) {
                //gpio data port
                if (ports_check.gpio_ports[position_feedback_config.biss_config.data_port_number-ENCODER_PORT_EXT_D0] != POSITION_FEEDBACK_PORTS_1 ||
                        (!isnull(position_feedback_config_2) && (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR) ) )
                {
                    position_feedback_config.sensor_type = 0;
                } else {
                    set_port_use_on(*gpio_ports[position_feedback_config.biss_config.data_port_number-ENCODER_PORT_EXT_D0]);
                    configure_out_port(*gpio_ports[position_feedback_config.biss_config.data_port_number-ENCODER_PORT_EXT_D0], spi_ports->spi_interface.blk1, 1);
                    position_feedback_config.gpio_config[position_feedback_config.biss_config.data_port_number-ENCODER_PORT_EXT_D0] = GPIO_OFF; //disable GPIO on this port
                }
            }
            set_hall_enc_select_config(hall_enc_select_config, position_feedback_config.biss_config.data_port_number, position_feedback_config.biss_config.data_port_signal_type);
            //check and configure clock port
            if (position_feedback_config.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4) { //hall_enc_select_port clock port
                if (ports_check.hall_enc_select_port != 1) {
                    position_feedback_config.sensor_type = 0;
                } else {
                    configure_out_port(*hall_enc_select_port, spi_ports->spi_interface.blk1, hall_enc_select_config);
                }
            } else { //gpio clock port
                if (ports_check.gpio_ports[position_feedback_config.biss_config.clock_port_config] != POSITION_FEEDBACK_PORTS_1 ||
                    (!isnull(position_feedback_config_2) && (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR) ) )
                {
                    position_feedback_config.sensor_type = 0;
                } else {
                    set_port_use_on(*gpio_ports[position_feedback_config.biss_config.clock_port_config]);
                    configure_out_port(*gpio_ports[position_feedback_config.biss_config.clock_port_config], spi_ports->spi_interface.blk1, 1);
                    position_feedback_config.gpio_config[position_feedback_config.biss_config.clock_port_config] = GPIO_OFF; //disable GPIO on this port
                }
            }
            //configure clock rate
            set_clock_on(spi_ports->spi_interface.blk1);
            configure_clock_rate_at_most(spi_ports->spi_interface.blk1, position_feedback_config.ifm_usec, ((position_feedback_config.ifm_usec*1000)/(position_feedback_config.biss_config.clock_frequency*2))+1);
            start_clock(spi_ports->spi_interface.blk1);
        }
    }
    if (ports_check.hall_enc_select_port == 1) {
        *hall_enc_select_port <: (hall_enc_select_config ^ hall_enc_select_port_inv_mask);
    }

}

void reset_ports(port * qei_hall_port_1, port * qei_hall_port_2, port * hall_enc_select_port, SPIPorts * spi_ports, port * gpio_ports[4], PositionFeedbackPortsCheck ports_check)
{
    if (ports_check.spi_ports == POSITION_FEEDBACK_PORTS_1) {
        set_clock_on(spi_ports->spi_interface.blk2);
        set_clock_on(spi_ports->spi_interface.blk1);
    }

    for (int i=0 ; i<4 ; i++) {
        if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_1) {
            set_port_use_on(*gpio_ports[i]);
        }
    }

    if (ports_check.qei_hall_port_1 == POSITION_FEEDBACK_PORTS_1) {
        set_port_use_on(*qei_hall_port_1);
    }
    if (ports_check.qei_hall_port_2 == POSITION_FEEDBACK_PORTS_1) {
        set_port_use_on(*qei_hall_port_2);
    }
    if (ports_check.hall_enc_select_port == POSITION_FEEDBACK_PORTS_1) {
        set_port_use_on(*hall_enc_select_port);
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

void write_shared_memory(client interface shared_memory_interface ?i_shared_memory, SensorFunction sensor_function, int count, int position, int velocity, int angle, int hall_state, unsigned int qei_index_found, SensorError sensor_error, SensorError last_sensor_error, unsigned int timestamp)
{
    if (!isnull(i_shared_memory)) {
        switch(sensor_function)
        {
        case SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL:
            i_shared_memory.write_angle_and_primary_feedback(angle, hall_state, qei_index_found, count, position, velocity, sensor_error, last_sensor_error, timestamp);
            break;
        case SENSOR_FUNCTION_COMMUTATION_AND_FEEDBACK_DISPLAY_ONLY:
            i_shared_memory.write_angle_and_secondary_feedback(angle, hall_state, qei_index_found, count, position, velocity, sensor_error, last_sensor_error, timestamp);
            break;
        case SENSOR_FUNCTION_MOTION_CONTROL:
            i_shared_memory.write_primary_feedback(count, position, velocity, sensor_error, last_sensor_error, timestamp);
            break;
        case SENSOR_FUNCTION_FEEDBACK_DISPLAY_ONLY:
            i_shared_memory.write_secondary_feedback(count, position, velocity, sensor_error, last_sensor_error, timestamp);
            break;
        case SENSOR_FUNCTION_COMMUTATION_ONLY:
            i_shared_memory.write_angle(angle, hall_state, qei_index_found, velocity, sensor_error, last_sensor_error);
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

void gpio_shared_memory(port * (&?gpio_ports)[4], PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, int gpio_on)
{
    if (gpio_on == 1 && !isnull(i_shared_memory)) {
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


int write_hall_state_angle_shared_memory(PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory)
{
    if (!isnull(i_shared_memory)) {
        if (position_feedback_config.sensor_function == SENSOR_FUNCTION_COMMUTATION_AND_MOTION_CONTROL ||
            position_feedback_config.sensor_function == SENSOR_FUNCTION_COMMUTATION_AND_FEEDBACK_DISPLAY_ONLY ||
            position_feedback_config.sensor_function == SENSOR_FUNCTION_COMMUTATION_ONLY)
        {
            i_shared_memory.write_hall_state_angle(position_feedback_config.hall_config.hall_state_angle);
            return 1;
        }
    }
    return 0;
}

void position_feedback_service(port ?qei_hall_port_1, port ?qei_hall_port_2, HallEncSelectPort &?hall_enc_select_port_struct, SPIPorts &?spi_ports, port ?gpio_port_0, port ?gpio_port_1, port ?gpio_port_2, port ?gpio_port_3,
                               PositionFeedbackConfig &position_feedback_config_1,
                               client interface shared_memory_interface ?i_shared_memory_1,
                               server interface PositionFeedbackInterface i_position_feedback_1[3],
                               PositionFeedbackConfig &?position_feedback_config_2,
                               client interface shared_memory_interface ?i_shared_memory_2,
                               server interface PositionFeedbackInterface (&?i_position_feedback_2)[3])
{

    timer t;
    unsigned ts;

    //proper task startup
    t :> ts;
    t when timerafter (ts + (1000*100*30)) :> void;

    if (position_feedback_config_1.ifm_usec == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    PositionFeedbackPortsCheck ports_check = {POSITION_FEEDBACK_PORTS_NULL};

    //check all ports
    //hall_enc_select_port
    unsigned int hall_enc_select_port_inv_mask = 0;
    if (!isnull(hall_enc_select_port_struct)) {
        if (!isnull(hall_enc_select_port_struct.p_hall_enc_select)) {
            ports_check.hall_enc_select_port = POSITION_FEEDBACK_PORTS_1;
            if (hall_enc_select_port_struct.hall_enc_select_inverted == 1) {
                hall_enc_select_port_inv_mask = 0b0011; //DC1K d1 has inverted logic
            }
        }
    }
    //gpio ports
    ports_check.gpio_ports[0] = POSITION_FEEDBACK_PORTS_NULL;
    ports_check.gpio_ports[1] = POSITION_FEEDBACK_PORTS_NULL;
    ports_check.gpio_ports[2] = POSITION_FEEDBACK_PORTS_NULL;
    ports_check.gpio_ports[3] = POSITION_FEEDBACK_PORTS_NULL;
    if (!isnull(gpio_port_0)) {
        ports_check.gpio_ports[0] = POSITION_FEEDBACK_PORTS_1;
    }
    if (!isnull(gpio_port_1)) {
        ports_check.gpio_ports[1] = POSITION_FEEDBACK_PORTS_1;
    }
    if (!isnull(gpio_port_2)) {
        ports_check.gpio_ports[2] = POSITION_FEEDBACK_PORTS_1;
    }
    if (!isnull(gpio_port_3)) {
        ports_check.gpio_ports[3] = POSITION_FEEDBACK_PORTS_1;
    }
    //qei hall ports
    if (!isnull(qei_hall_port_1)) {
        ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_1;
    }
    if (!isnull(qei_hall_port_2)) {
        ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_1;
    }
    //spi ports
    if (!isnull(spi_ports)) {
        if ((!isnull(spi_ports.spi_interface.blk1)) && (!isnull(spi_ports.spi_interface.blk2))) {
            ports_check.spi_ports = POSITION_FEEDBACK_PORTS_1;
        }
    }

    //move hall_enc_select_port
    port * movable hall_enc_select_port;
    HallEncSelectPort * movable hall_enc_select_port_struct_p = &hall_enc_select_port_struct;
    port ? * movable hall_enc_select_port_temp =  &(hall_enc_select_port_struct_p->p_hall_enc_select);
    if (ports_check.hall_enc_select_port == POSITION_FEEDBACK_PORTS_1) {
        hall_enc_select_port = reconfigure_port(move(hall_enc_select_port_temp), port);
    }

    //move gpio ports
    port ? * movable gpio_0 = &gpio_port_0;
    port ? * movable gpio_1 = &gpio_port_1;
    port ? * movable gpio_2 = &gpio_port_2;
    port ? * movable gpio_3 = &gpio_port_3;
    port * movable gpio_ports[4];
    if (ports_check.gpio_ports[0] == POSITION_FEEDBACK_PORTS_1) {
        gpio_ports[0] = reconfigure_port(move(gpio_0), port);
    }
    if (ports_check.gpio_ports[1] == POSITION_FEEDBACK_PORTS_1) {
        gpio_ports[1] = reconfigure_port(move(gpio_1), port);
    }
    if (ports_check.gpio_ports[2] == POSITION_FEEDBACK_PORTS_1) {
        gpio_ports[2] = reconfigure_port(move(gpio_2), port);
    }
    if (ports_check.gpio_ports[3] == POSITION_FEEDBACK_PORTS_1) {
        gpio_ports[3] = reconfigure_port(move(gpio_3), port);
    }

    //pointers to ports 1
    port ? * movable qei_hall_port_1_temp = &qei_hall_port_1;
    port ? * movable qei_hall_port_2_temp = &qei_hall_port_2;
    port * movable qei_hall_port_1_1;
    port * movable qei_hall_port_2_1;
    if (ports_check.qei_hall_port_1 == POSITION_FEEDBACK_PORTS_1) {
        qei_hall_port_1_1 = reconfigure_port(move(qei_hall_port_1_temp), port);
    }
    if (ports_check.qei_hall_port_2 == POSITION_FEEDBACK_PORTS_1) {
        qei_hall_port_2_1 = reconfigure_port(move(qei_hall_port_2_temp), port);
    }
    port * movable biss_data_port_1;
    port * movable biss_clock_port_1;
    SPIPorts * movable spi_ports_1 = &spi_ports;

    //pointers to ports 2
    port * movable qei_hall_port_1_2;
    port * movable qei_hall_port_2_2;
    port * movable biss_clock_port_2;
    port * movable biss_data_port_2;
    SPIPorts * movable spi_ports_2;

    int spi_on = 0;
    unsigned int biss_clock_low_1 = 0;
    unsigned int biss_clock_high_1 = 1;
    unsigned int biss_clock_low_2 = 0;
    unsigned int biss_clock_high_2 = 1;

    while(1) {
        //reset clocks and ports
        reset_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port, spi_ports_1, gpio_ports, ports_check);

        //config gpio ports
        for (int i=0 ; i<4 ; i++) {
            if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_1) {
                if (position_feedback_config_1.gpio_config[i] == GPIO_INPUT_PULLDOWN) {
                    set_port_pull_down(*gpio_ports[i]);
                } else {
                    set_port_pull_none(*gpio_ports[i]);
                }
            }
        }

        //write hall state angle to shared memory for sensor 1 if used for commutation
        int hall_state_angle_written = 0;
        hall_state_angle_written = write_hall_state_angle_shared_memory(position_feedback_config_1, i_shared_memory_1);
        hall_state_angle_written = write_hall_state_angle_shared_memory(position_feedback_config_1, i_shared_memory_2);

        int hall_enc_select_config = 0b0011; // set qei_hall_port_1 and 2 to RS422 (differential) mode

        //checks ports sensor 2
        if (!isnull(position_feedback_config_2)) {
            //check an configure ports, set to fallback service if incorrect configuration
            check_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port, spi_ports_1, gpio_ports, ports_check, hall_enc_select_config, hall_enc_select_port_inv_mask, position_feedback_config_2, position_feedback_config_1);

            //move all needed ports to ports pointers number 2
            switch(position_feedback_config_2.sensor_type) {
            case HALL_SENSOR:
                if (position_feedback_config_2.hall_config.port_number == ENCODER_PORT_1) {
                    qei_hall_port_1_2 = move(qei_hall_port_1_1);
                    ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_2;
                } else if (position_feedback_config_2.hall_config.port_number == ENCODER_PORT_2) {
                    qei_hall_port_2_2 = move(qei_hall_port_2_1);
                    ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_2;
                }
                break;
            case QEI_SENSOR:
                if (position_feedback_config_2.qei_config.port_number == ENCODER_PORT_1) {
                    qei_hall_port_1_2 = move(qei_hall_port_1_1);
                    ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_2;
                } else if (position_feedback_config_2.qei_config.port_number == ENCODER_PORT_2) {
                    qei_hall_port_2_2 = move(qei_hall_port_2_1);
                    ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_2;
                }
                break;
            case BISS_SENSOR:
            case SSI_SENSOR:
                //move data port
                if (position_feedback_config_2.biss_config.data_port_number == ENCODER_PORT_1) {
                    biss_data_port_2 = reconfigure_port(move(qei_hall_port_1_1),port);
                    ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_BISS_DATA_2;
                } else if (position_feedback_config_2.biss_config.data_port_number == ENCODER_PORT_2) {
                    biss_data_port_2 = move(qei_hall_port_2_1);
                    ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_BISS_DATA_2;
                } else if (position_feedback_config_2.biss_config.data_port_number >= ENCODER_PORT_EXT_D0 && position_feedback_config_2.biss_config.data_port_number <= ENCODER_PORT_EXT_D3) {
                    ports_check.gpio_ports[position_feedback_config_2.biss_config.data_port_number-ENCODER_PORT_EXT_D0] = POSITION_FEEDBACK_PORTS_BISS_DATA_2; //mark that we move the port
                    biss_data_port_2 = move(gpio_ports[position_feedback_config_2.biss_config.data_port_number-ENCODER_PORT_EXT_D0]);
                }
                //move clock port
                if (position_feedback_config_2.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4) { //hall_enc_select_port clock port
                    biss_clock_port_2 = move(hall_enc_select_port);
                    ports_check.hall_enc_select_port = POSITION_FEEDBACK_PORTS_BISS_CLOCK_2; //mark that we move hall_enc_select_port
                } else { //gpio clock port
                    biss_clock_port_2 = move(gpio_ports[position_feedback_config_2.biss_config.clock_port_config]);
                    ports_check.gpio_ports[position_feedback_config_2.biss_config.clock_port_config] = POSITION_FEEDBACK_PORTS_BISS_CLOCK_2; //mark that we move the port
                    biss_clock_low_2 = 0;
                    biss_clock_high_2 = 1;
                }
                break;
            }

            //write hall state angle to shared memory if sensor 2 is used for commutation
            if (hall_state_angle_written == 0) {
                hall_state_angle_written = write_hall_state_angle_shared_memory(position_feedback_config_2, i_shared_memory_1);
                hall_state_angle_written = write_hall_state_angle_shared_memory(position_feedback_config_2, i_shared_memory_2);
            }
        }

        //checks ports sensor 1
        //check an configure ports, set to fallback service if incorrect configuration
        check_ports(qei_hall_port_1_1, qei_hall_port_2_1, hall_enc_select_port, spi_ports_1, gpio_ports, ports_check, hall_enc_select_config, hall_enc_select_port_inv_mask, position_feedback_config_1, position_feedback_config_2);

        //move biss ports for service 1
        if (position_feedback_config_1.sensor_type == BISS_SENSOR || position_feedback_config_1.sensor_type == SSI_SENSOR) {
            //move data port
            if (position_feedback_config_1.biss_config.data_port_number == ENCODER_PORT_1) {
                biss_data_port_1 = move(qei_hall_port_1_1);
                ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_BISS_DATA_1;
            } else if (position_feedback_config_1.biss_config.data_port_number == ENCODER_PORT_2) {
                biss_data_port_1 = move(qei_hall_port_2_1);
                ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_BISS_DATA_1;
            } else if (position_feedback_config_1.biss_config.data_port_number >= ENCODER_PORT_EXT_D0 && position_feedback_config_1.biss_config.data_port_number <= ENCODER_PORT_EXT_D3) {
                ports_check.gpio_ports[position_feedback_config_1.biss_config.data_port_number-ENCODER_PORT_EXT_D0] = POSITION_FEEDBACK_PORTS_BISS_DATA_1; //mark that we move the port
                biss_data_port_1 = move(gpio_ports[position_feedback_config_1.biss_config.data_port_number-ENCODER_PORT_EXT_D0]);
            }
            //move clock port
            if (position_feedback_config_1.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4) { //hall_enc_select_port clock port
                biss_clock_port_1 = move(hall_enc_select_port);
                ports_check.hall_enc_select_port = POSITION_FEEDBACK_PORTS_BISS_CLOCK_1; //mark that we move hall_enc_select_port
                biss_clock_low_1 = hall_enc_select_config ^ hall_enc_select_port_inv_mask;
                biss_clock_high_1 = position_feedback_config_1.biss_config.clock_port_config | (hall_enc_select_config ^ hall_enc_select_port_inv_mask);
            } else { //gpio clock port
                biss_clock_port_1 = move(gpio_ports[position_feedback_config_1.biss_config.clock_port_config]);
                ports_check.gpio_ports[position_feedback_config_1.biss_config.clock_port_config] = POSITION_FEEDBACK_PORTS_BISS_CLOCK_1; //mark that we move the port
                biss_clock_low_1 = 0;
                biss_clock_high_1 = 1;
            }
        }

        //move gpio ports to spi ports if needed, move spi ports to service 2 if needed
        if ( position_feedback_config_1.sensor_type == REM_16MT_SENSOR || position_feedback_config_1.sensor_type == REM_14_SENSOR)
        {
            set_clock_on(spi_ports_1->spi_interface.blk1);
            spi_ports_1->slave_select = reconfigure_port(move(gpio_ports[0]), port);
            spi_ports_1->spi_interface.sclk = reconfigure_port(move(gpio_ports[1]), out buffered port:8);
            spi_ports_1->spi_interface.miso = reconfigure_port(move(gpio_ports[2]), in buffered port:8);
            spi_ports_1->spi_interface.mosi = reconfigure_port(move(gpio_ports[3]), out buffered port:8);
            spi_on = 1;
        }
        if (!isnull(position_feedback_config_2)) {
            if (position_feedback_config_2.sensor_type == REM_16MT_SENSOR || position_feedback_config_2.sensor_type == REM_14_SENSOR)
            {
                set_clock_on(spi_ports_1->spi_interface.blk1);
                spi_ports_1->slave_select = reconfigure_port(move(gpio_ports[0]), port);
                spi_ports_1->spi_interface.sclk = reconfigure_port(move(gpio_ports[1]), out buffered port:8);
                spi_ports_1->spi_interface.miso = reconfigure_port(move(gpio_ports[2]), in buffered port:8);
                spi_ports_1->spi_interface.mosi = reconfigure_port(move(gpio_ports[3]), out buffered port:8);
                spi_ports_2 = move(spi_ports_1);
                ports_check.spi_ports = POSITION_FEEDBACK_PORTS_2;
                spi_on = 1;
            }
            //set biss_clock_low and high for service 2 only after both services are configured
            else if ((position_feedback_config_2.sensor_type == BISS_SENSOR || position_feedback_config_2.sensor_type == SSI_SENSOR) &&
                      position_feedback_config_2.biss_config.clock_port_config >= BISS_CLOCK_PORT_EXT_D4)
            {
                biss_clock_low_2 = hall_enc_select_config ^ hall_enc_select_port_inv_mask;
                biss_clock_high_2 = position_feedback_config_2.biss_config.clock_port_config | (hall_enc_select_config ^ hall_enc_select_port_inv_mask);
            }

        }

        //write hall state angle to shared memory for sensor 1 if not written before
        if (hall_state_angle_written == 0) {
            if (!isnull(i_shared_memory_1)) {
                i_shared_memory_1.write_hall_state_angle(position_feedback_config_1.hall_config.hall_state_angle);
            } else if (!isnull(i_shared_memory_2)) {
                i_shared_memory_2.write_hall_state_angle(position_feedback_config_1.hall_config.hall_state_angle);
            }
        }

        //start services
        par {
            {//sensor 1
                start_service(qei_hall_port_1_1, qei_hall_port_2_1, biss_clock_port_1, biss_data_port_1, spi_ports_1, gpio_ports, biss_clock_low_1, biss_clock_high_1, position_feedback_config_1, i_shared_memory_1, i_position_feedback_1, 1);
            }
            {//sensor 2
                if (!isnull(i_position_feedback_2) && !isnull(position_feedback_config_2)) {
                    start_service(qei_hall_port_1_2, qei_hall_port_2_2, biss_clock_port_2, biss_data_port_2, spi_ports_2, null, biss_clock_low_2, biss_clock_high_2, position_feedback_config_2, i_shared_memory_2, i_position_feedback_2, 0);
                }
            }
        }

        //move back ports
        //qei_hall_port_1
        if (ports_check.qei_hall_port_1 == POSITION_FEEDBACK_PORTS_BISS_DATA_1) {
            qei_hall_port_1_1 = move(biss_data_port_1);
            ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_1;
        } else if (ports_check.qei_hall_port_1 == POSITION_FEEDBACK_PORTS_BISS_DATA_2) {
            qei_hall_port_1_1 = move(biss_data_port_2);
            ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_1;
        } else if (ports_check.qei_hall_port_1 == POSITION_FEEDBACK_PORTS_2) {
            qei_hall_port_1_1 = move(qei_hall_port_1_2);
            ports_check.qei_hall_port_1 = POSITION_FEEDBACK_PORTS_1;
        }
        //qei_hall_port_2
        if (ports_check.qei_hall_port_2 == POSITION_FEEDBACK_PORTS_BISS_DATA_1) {
            qei_hall_port_2_1 = move(biss_data_port_1);
            ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_1;
        } else if (ports_check.qei_hall_port_2 == POSITION_FEEDBACK_PORTS_BISS_DATA_2) {
            qei_hall_port_2_1 = move(biss_data_port_2);
            ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_1;
        } else if (ports_check.qei_hall_port_2 == POSITION_FEEDBACK_PORTS_2) {
            qei_hall_port_2_1 = move(qei_hall_port_2_2);
            ports_check.qei_hall_port_2 = POSITION_FEEDBACK_PORTS_1;
        }
        //hall_enc_select_port
        if (ports_check.hall_enc_select_port == POSITION_FEEDBACK_PORTS_BISS_CLOCK_1) {
            hall_enc_select_port = move(biss_clock_port_1);
            ports_check.hall_enc_select_port = POSITION_FEEDBACK_PORTS_1;
        } else if (ports_check.hall_enc_select_port == POSITION_FEEDBACK_PORTS_BISS_CLOCK_2) {
            hall_enc_select_port = move(biss_clock_port_2);
            ports_check.hall_enc_select_port = POSITION_FEEDBACK_PORTS_1;
        }
        //gpio ports
        for (int i=0 ; i<4 ; i++) {
            if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_BISS_DATA_1) {
                gpio_ports[i] = move(biss_data_port_1);
                ports_check.gpio_ports[i] = POSITION_FEEDBACK_PORTS_1;
            } else if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_BISS_DATA_2) {
                gpio_ports[i] = move(biss_data_port_2);
                ports_check.gpio_ports[i] = POSITION_FEEDBACK_PORTS_1;
            } else if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_BISS_CLOCK_1) {
                gpio_ports[i] = move(biss_clock_port_1);
                ports_check.gpio_ports[i] = POSITION_FEEDBACK_PORTS_1;
            } else if (ports_check.gpio_ports[i] == POSITION_FEEDBACK_PORTS_BISS_CLOCK_2) {
                gpio_ports[i] = move(biss_clock_port_2);
                ports_check.gpio_ports[i] = POSITION_FEEDBACK_PORTS_1;
            }
        }
        //spi ports
        if (ports_check.spi_ports == POSITION_FEEDBACK_PORTS_2) {
            spi_ports_1 = move(spi_ports_2);
            ports_check.spi_ports = POSITION_FEEDBACK_PORTS_1;
        }
        if (spi_on) {
            gpio_ports[0] = reconfigure_port(move(spi_ports_1->slave_select), port);
            gpio_ports[1] = reconfigure_port(move(spi_ports_1->spi_interface.sclk), port);
            gpio_ports[2] = reconfigure_port(move(spi_ports_1->spi_interface.miso), port);
            gpio_ports[3] = reconfigure_port(move(spi_ports_1->spi_interface.mosi), port);
            spi_on = 0;
        }
    }
}
