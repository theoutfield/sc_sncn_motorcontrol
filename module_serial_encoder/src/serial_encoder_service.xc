/*
 * serial_encoder_service.xc
 *
 *  Created on: 26.11.2016
 *      Author: synapticon
 */

#include <xs1.h>
#include <rem_16mt_service.h>
#include <rem_14_service.h>
#include <biss_service.h>
#include <timer.h>
#include <print.h>
#include <xscope.h>
#include <mc_internal_constants.h>
#include <filter_blocks.h>

extern char start_message[];

typedef struct {
    int count;
    unsigned int position;
    unsigned int last_position;
    unsigned int angle;
    unsigned int status;
    unsigned int timestamp;
} PositionState;


void read_position(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * biss_clock_port, int hall_enc_select_config, PositionFeedbackConfig &position_feedback_config, PositionState &state,
        timer t, unsigned int &last_read)
{
    state.status = -1;

    switch(position_feedback_config.sensor_type)
    {
    case REM_16MT_SENSOR:
        t when timerafter(last_read + position_feedback_config.rem_16mt_config.timeout) :> void;
#ifdef REM_16MT_USE_TIMESTAMP
        char timestamp;
        { state.status, state.count, state.position, state.angle, state.timestamp } = rem_16mt_read(*spi_ports);
#else
        { state.status, state.count, state.position, state.angle } = rem_16mt_read(*spi_ports);
#endif
        state.angle = (position_feedback_config.pole_pairs * (state.angle >> 4) ) & 4095;
        break;
    case REM_14_SENSOR:
        state.position = readRotarySensorAngleWithoutCompensation(*spi_ports);
        state.status = 0;
        multiturn(state.count, state.last_position, state.position, position_feedback_config.resolution);
        state.angle = (position_feedback_config.pole_pairs * (state.position >> 2) ) & 4095;
        break;
    case BISS_SENSOR:
        unsigned int data[BISS_FRAME_BYTES];
        t when timerafter(last_read + position_feedback_config.biss_config.timeout) :> void;
        int error = read_biss_sensor_data(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, hall_enc_select_config, biss_clock_port, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
        { state.count, state.position, state.status } = biss_encoder(data, position_feedback_config.biss_config);
        state.status = state.status + (error << 2);
        if (position_feedback_config.biss_config.multiturn_resolution == 0)
        {
            multiturn(state.count, state.last_position, state.position, position_feedback_config.resolution);
        }
        if (position_feedback_config.polarity == BISS_POLARITY_INVERTED) {
            state.count = -state.count;
            state.position = position_feedback_config.resolution - state.position - 1;
        }
        if (position_feedback_config.biss_config.singleturn_resolution > 12) {
            state.angle = (position_feedback_config.pole_pairs * (state.position >> (position_feedback_config.biss_config.singleturn_resolution-12))) & 4095;
        } else {
            state.angle = (position_feedback_config.pole_pairs * (state.position << (12-position_feedback_config.biss_config.singleturn_resolution))) & 4095;
        }
        break;
    }
    state.last_position = state.position;
    t :> last_read;
    return;
}

int init_sensor(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * biss_clock_port,
        int hall_enc_select_config, PositionFeedbackConfig &position_feedback_config, PositionState &pos_state,
        timer t, unsigned int &last_read)
{
    //init
    int velocity_loop;
    position_feedback_config.offset &= (position_feedback_config.resolution-1);
    switch(position_feedback_config.sensor_type)
    {
    case REM_16MT_SENSOR:
        velocity_loop = position_feedback_config.rem_16mt_config.velocity_loop * REM_16MT_USEC; //velocity loop time in clock ticks
        //init sensor
        init_spi_ports(*spi_ports);
        rem_16mt_init(*spi_ports, position_feedback_config);
        break;
    case REM_14_SENSOR:
        velocity_loop = position_feedback_config.rem_14_config.velocity_loop * REM_14_USEC; //velocity loop time in clock ticks
        init_spi_ports(*spi_ports);
        initRotarySensor(*spi_ports,  position_feedback_config);
        break;
    case BISS_SENSOR:
        //check if resolution is a power of 2
        if ( position_feedback_config.resolution & (position_feedback_config.resolution -1) )
        {
#ifdef DEBUG_POSITION_FEEDBACK
            printstrln("BISS service: Wrong resolution");
#endif
            position_feedback_config.sensor_type = 0;
//            return;
        }
        position_feedback_config.biss_config.singleturn_resolution = tickstobits(position_feedback_config.resolution);
        velocity_loop = (position_feedback_config.biss_config.velocity_loop * BISS_USEC); //velocity loop time in clock ticks
        break;
    }
    //read
    read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, biss_clock_port, hall_enc_select_config, position_feedback_config, pos_state, t, last_read);


#ifdef DEBUG_POSITION_FEEDBACK
    switch(position_feedback_config.sensor_type)
    {
    case BISS_SENSOR:
        int init_status = pos_state.status >> 2;
        if (init_status == CRCError)
            printstrln("biss_service: ERROR: CRC");
        else if (init_status == NoStartBit)
            printstrln("biss_service: ERROR: No Start bit");
        else if (init_status == NoAck)
            printstrln("biss_service: ERROR: No Ack bit");
        else if (init_status != NoError)
            printstrln("biss_service: ERROR: initialization");
        printstr(start_message);
        printstrln("BISS");
        break;
    }

#endif


    return velocity_loop;
}


void serial_encoder_service(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4], int hall_enc_select_config, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3])
{
    if (REM_16MT_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }


    //init variables
    //velocity
    int velocity = 0;
    int velocity_buffer[8] = {0};
    int index = 0;
    int old_count = 0;
    int crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
    int velocity_count = 0;
    int velocity_factor = 60000000/position_feedback_config.rem_14_config.velocity_loop;
    int velocity_loop;
#ifdef REM_16MT_USE_TIMESTAMP
    char old_timestamp = 0, timediff;
    int timediff_long = 0;
#endif
    //position
    PositionState pos_state = {0};
    //timing
    timer t;
    unsigned int last_read = 0;
    t :> last_read;
    unsigned int last_velocity_read = last_read;
    unsigned int next_velocity_read = last_read;
    unsigned int end_time = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    velocity_loop = init_sensor(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);

#if 0
    //init sensor
    switch(position_feedback_config.sensor_type)
    {
    case REM_16MT_SENSOR:
        velocity_loop = position_feedback_config.rem_16mt_config.velocity_loop * REM_16MT_USEC; //velocity loop time in clock ticks
        //init sensor
        init_spi_ports(*spi_ports);
        int init_status = rem_16mt_init(*spi_ports, position_feedback_config);
#ifdef DEBUG_POSITION_FEEDBACK
        if (init_status) {
            delay_ticks(200000*REM_16MT_USEC);
            init_status = rem_16mt_init(*spi_ports, position_feedback_config);
            if (init_status) {
                printstr("Error with REM_16MT sensor initialization");
                printintln(init_status);
            }
        }
        printstr(start_message);
        printstrln("REM_16MT");
#endif
        //first read
        delay_ticks(100 * REM_16MT_USEC);
    #ifdef REM_16MT_USE_TIMESTAMP
        { void, pos_state.count, pos_state.last_position, void, void } = rem_16mt_read(*spi_ports);
    #else
        { void, pos_state.count, pos_state.last_position, void } = rem_16mt_read(*spi_ports);
    #endif
        t :> last_read;
        break;
    case REM_14_SENSOR:
        velocity_loop = position_feedback_config.rem_14_config.velocity_loop * REM_14_USEC; //velocity loop time in clock ticks
        position_feedback_config.offset &= (position_feedback_config.resolution-1);
#ifdef DEBUG_POSITION_FEEDBACK
        if (initRotarySensor(*spi_ports,  position_feedback_config) != SUCCESS_WRITING) {
            printstrln("Error with SPI REM_14 sensor");
        }
        printstr(start_message);
        printstrln("REM_14");
#endif
        pos_state.last_position = readRotarySensorAngleWithoutCompensation(*spi_ports);
        t :> last_read;
        break;
    case BISS_SENSOR:
        //check if resolution is a power of 2
        if ( position_feedback_config.resolution & (position_feedback_config.resolution -1) )
        {
#ifdef DEBUG_POSITION_FEEDBACK
            printstrln("BISS service: Wrong resolution");
#endif
            position_feedback_config.sensor_type = 0;
            return;
        }
        position_feedback_config.biss_config.singleturn_resolution = tickstobits(position_feedback_config.resolution);
        velocity_loop = (position_feedback_config.biss_config.velocity_loop * BISS_USEC); //velocity loop time in clock ticks
        velocity_factor = 60000000/position_feedback_config.biss_config.velocity_loop;

        //first read
        t :> last_read;
        unsigned time = last_read;
        unsigned int data[BISS_FRAME_BYTES];
        int init_status;
        do {
            t when timerafter(last_read + position_feedback_config.biss_config.timeout) :> void;
            init_status = read_biss_sensor_data(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, hall_enc_select_config, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
            t :> last_read;
        } while (init_status != NoError && !timeafter(last_read, time + 1000000*BISS_USEC));
#ifdef DEBUG_POSITION_FEEDBACK
        if (init_status == CRCError)
            printstrln("biss_service: ERROR: CRC");
        else if (init_status == NoStartBit)
            printstrln("biss_service: ERROR: No Start bit");
        else if (init_status == NoAck)
            printstrln("biss_service: ERROR: No Ack bit");
        else if (init_status != NoError)
            printstrln("biss_service: ERROR: initialization");
        printstr(start_message);
        printstrln("BISS");
#endif
        { void , pos_state.last_position, void } = biss_encoder(data, position_feedback_config.biss_config);
        break;
    }
#endif

    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
//                wait_sensor(position_feedback_config, t, last_read);
                read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
//                t :> last_read;
                angle = pos_state.angle;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position , unsigned int status }:
//                wait_sensor(position_feedback_config, t, last_read);
                read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
//                t :> last_read;
                out_count = pos_state.count;
                position = pos_state.position;
                status = pos_state.status;
                break;

#if 0
        //send position
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
//                wait_sensor(position_feedback_config, t, last_read);
                read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
//                t :> last_read;
                out_count = pos_state.count;
                position = pos_state.position;
                status = pos_state.status;
                break;
#endif

        //send velocity
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //send ticks per turn
//        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
//                out_ticks_per_turn = position_feedback_config.resolution;
//                break;

        //receive new rem_16mt_config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):

#if 0
                switch(position_feedback_config.sensor_type)
                {
                case REM_16MT_SENSOR:
                    position_feedback_config = in_config;
                    rem_16mt_init(*spi_ports, position_feedback_config); //init with new config
                    t :> last_read;
                    //update variables which depend on rem_16mt_config
                    velocity_loop = position_feedback_config.rem_16mt_config.velocity_loop * REM_16MT_USEC;
                    break;
                case REM_14_SENSOR:
                    in_config.offset &= (in_config.resolution-1);
                    //update variables which depend on rem_14_config
                    if (position_feedback_config.polarity != in_config.polarity)
                        initRotarySensor(*spi_ports,  in_config);
                    else if (position_feedback_config.offset != in_config.offset)
                        writeZeroPosition(*spi_ports, in_config.offset);
                    position_feedback_config = in_config;
                    velocity_loop = position_feedback_config.rem_14_config.velocity_loop * REM_14_USEC;
                    velocity_factor = 60000000/position_feedback_config.rem_14_config.velocity_loop;
                    break;
                case BISS_SENSOR:
                    //update variables which depend on biss_config
                    position_feedback_config = in_config;
                    position_feedback_config.biss_config.singleturn_resolution = tickstobits(position_feedback_config.resolution);
                    position_feedback_config.offset &= 4095;
//                    max_ticks_internal = (1 << (position_feedback_config.biss_config.multiturn_resolution -1 + position_feedback_config.biss_config.singleturn_resolution));
                    velocity_loop = (position_feedback_config.biss_config.velocity_loop * BISS_USEC);
                    velocity_factor = 60000000/position_feedback_config.biss_config.velocity_loop;
                    break;
                }
#endif
                velocity_loop = init_sensor(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
                crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                break;

        //send rem_16mt_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        //receive the new count to set
        case i_position_feedback[int i].set_position(int new_count):
                switch(position_feedback_config.sensor_type)
                {
                case REM_16MT_SENSOR:
                    int multiturn;
                    unsigned int singleturn;
                    if (new_count < 0) {
                        multiturn = (new_count / position_feedback_config.resolution) - 1;
                        singleturn = position_feedback_config.resolution + new_count % position_feedback_config.resolution;
                    } else {
                        multiturn = (new_count / position_feedback_config.resolution);
                        singleturn = new_count % position_feedback_config.resolution;
                    }
                    t when timerafter(last_read + position_feedback_config.rem_16mt_config.timeout) :> void;
                    rem_16mt_write(*spi_ports, REM_16MT_CONF_PRESET, (multiturn << 16) + singleturn, 32);
                    pos_state.last_position = singleturn;
                    break;
                case REM_14_SENSOR:
                    pos_state.last_position = readRotarySensorAngleWithoutCompensation(*spi_ports);
                    break;
                case BISS_SENSOR:
//                    t when timerafter(last_read + position_feedback_config.biss_config.timeout) :> void;
                    read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
                    break;
                }
                t :> last_read;
                pos_state.count = new_count;
                break;

#if 0
        //receive the new electrical angle to set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                switch(position_feedback_config.sensor_type)
                {
                case REM_16MT_SENSOR:
                    new_angle = (new_angle << 4);
                    t when timerafter(last_read + position_feedback_config.rem_16mt_config.timeout) :> void;
                    rem_16mt_write(*spi_ports, REM_16MT_CTRL_RESET, 0, 0);//reset
                    int real_position;
#ifdef REM_16MT_USE_TIMESTAMP
                    { void, void, real_position, void, void } = rem_16mt_read(*spi_ports);
                    delay_ticks(position_feedback_config.rem_16mt_config.timeout);
                    rem_16mt_write(*spi_ports, REM_16MT_CONF_STPRESET, new_angle / position_feedback_config.pole_pairs, 16);
                    { void, void, out_offset, void, void } = rem_16mt_read(*spi_ports);
#else
                    { void, void, real_position, void } = rem_16mt_read(*spi_ports);
                    delay_ticks(position_feedback_config.rem_16mt_config.timeout);
                    rem_16mt_write(*spi_ports, REM_16MT_CONF_STPRESET, new_angle / position_feedback_config.pole_pairs, 16);
                    { void, void, out_offset, void } = rem_16mt_read(*spi_ports);
#endif
                    out_offset = (out_offset - real_position) & (position_feedback_config.resolution-1);
                    break;
                case REM_14_SENSOR:
                    writeZeroPosition(*spi_ports, 0);
                    int position = readRotarySensorAngleWithoutCompensation(*spi_ports);
                    out_offset = (position_feedback_config.resolution - ((new_angle << 2) / position_feedback_config.pole_pairs) + position) & (position_feedback_config.resolution-1);
                    writeZeroPosition(*spi_ports, out_offset);
                    break;
                }
                t :> last_read;
                position_feedback_config.offset = out_offset;
                break;
#endif

        //execute command
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                if (position_feedback_config.sensor_type == REM_16MT_SENSOR)
                {
                    t when timerafter(last_read + position_feedback_config.rem_16mt_config.timeout) :> void;
                    rem_16mt_write(*spi_ports, opcode, data, data_bits);
#ifdef REM_16MT_USE_TIMESTAMP
                    { status, void, void, void, void } = rem_16mt_read(*spi_ports);
#else
                    { status, void, void, void } = rem_16mt_read(*spi_ports);
#endif
                    t :> last_read;
                }
                break;

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

        //compute velocity
        case t when timerafter(next_velocity_read) :> next_velocity_read:
            next_velocity_read += velocity_loop;
            int difference;
//            wait_sensor(position_feedback_config, t, last_read);
            read_position(qei_hall_port_1, qei_hall_port_2, hall_enc_select_port, spi_ports, gpio_ports[position_feedback_config.biss_config.clock_port_config & 0b11], hall_enc_select_config, position_feedback_config, pos_state, t, last_read);
//            t :> last_read;

            switch(position_feedback_config.sensor_type)
            {
            case REM_16MT_SENSOR:
                velocity_count++;
#ifdef REM_16MT_USE_TIMESTAMP
                //timestamp difference
                timediff = (char)pos_state.timestamp-old_timestamp;
                old_timestamp = pos_state.timestamp;

                //velocity 8 samples (424us), average filter 8
                timediff_long += timediff;
                if (velocity_count >= 8) {
                    difference = pos_state.count - old_count;
                    old_count = pos_state.count;
                    if (timediff_long != 0 && difference < crossover && difference > -crossover) {
                        velocity = (difference * (60000000/timediff_long)) / position_feedback_config.resolution;
                        velocity = filter(velocity_buffer, index, 8, velocity);
                    }
                    timediff_long = 0;
                    velocity_count = 0;
                }
#else
                if (velocity_count >= 8) {
                    difference = count - old_count;
                    old_count = count;
                    if (last_read != last_velocity_read && difference < crossover && difference > -crossover) {
                        velocity = (difference * (60000000/((int)(last_read-last_velocity_read)/REM_16MT_USEC))) / position_feedback_config.resolution;
                        velocity = filter(velocity_buffer, index, 8, velocity);
                    }
                    last_velocity_read = last_read;
                    velocity_count = 0;
                }
#endif
                break;
            case REM_14_SENSOR:
            case BISS_SENSOR:
                difference = pos_state.count - old_count;
                old_count = pos_state.count;
                // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
                //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
                if (last_read != last_velocity_read && difference < crossover && difference > -crossover) {
                    velocity = (difference * (60000000/((int)(last_read-last_velocity_read)/REM_14_USEC))) / position_feedback_config.resolution;
                }
                last_velocity_read = last_read;
                break;
            }


#ifdef XSCOPE_REM_16MT
            xscope_int(VELOCITY, velocity);
            xscope_int(POSITION, pos_state.count);
            xscope_int(POSITION_RAW, pos_state.angle);
            xscope_int(STATUS, pos_state.status*1000);
#ifdef REM_16MT_USE_TIMESTAMP
            xscope_int(TIMESTAMP, timediff);
#endif
#endif

            //send data to shared memory
            if (!isnull(i_shared_memory)) {
                if (position_feedback_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(pos_state.angle, velocity, pos_state.count);
                } else if (position_feedback_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(pos_state.angle);
                } else if (position_feedback_config.enable_push_service == PushPosition) {
                    i_shared_memory.write_velocity_position(velocity, pos_state.count);
                }
            }

            //gpio
            gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory);

            //to prevent blocking
            t :> end_time;
            if (timeafter(end_time, next_velocity_read)) {
                next_velocity_read = end_time + REM_16MT_USEC;
            }
            break;
        }
    }
}

