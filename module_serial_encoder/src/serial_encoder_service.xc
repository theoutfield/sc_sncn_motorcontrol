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
#include <filters.h>
#include <stdio.h>

extern char start_message[];

typedef struct {
    int count;
    unsigned int position;
    unsigned int last_position;
    unsigned int angle;
    SensorError status;
    unsigned int timestamp;
} PositionState;


void read_position(port * biss_clock_port, port * biss_data_port, SPIPorts * spi_ports,
        int biss_clock_low, int biss_clock_high,
        PositionFeedbackConfig &position_feedback_config, int sensor_type, PositionState &state,
        timer t, unsigned int &last_read)
{
    switch(sensor_type)
    {
    case REM_16MT_SENSOR:
        t when timerafter(last_read + REM_16MT_TIMEOUT*position_feedback_config.ifm_usec) :> void;
        { state.status, state.count, state.position, state.angle, state.timestamp } = rem_16mt_read(*spi_ports, position_feedback_config.ifm_usec);
        t :> last_read;
#ifdef XSCOPE_REM_16MT
            xscope_int(POSITION_RAW, state.angle);
#endif
        state.angle = (position_feedback_config.pole_pairs * (state.angle >> 4) ) & 4095;
        break;
    case REM_14_SENSOR:
        { state.position,state.status } = readRotarySensorAngleWithoutCompensation(*spi_ports, position_feedback_config.ifm_usec);
        t :> last_read;
        multiturn(state.count, state.last_position, state.position, position_feedback_config.resolution);
        state.angle = (position_feedback_config.pole_pairs * (state.position >> 2) ) & 4095;
        break;
    case BISS_SENSOR:
    case SSI_SENSOR:
        unsigned int data[BISS_FRAME_BYTES];
        state.status = read_biss_sensor_data(biss_clock_port, biss_data_port, biss_clock_low, biss_clock_high, t, position_feedback_config, data);
        t :> last_read;
        int count;
        if(state.status == SENSOR_NO_ERROR) {
            { count, state.position, state.status } = biss_encoder(data, position_feedback_config);
        } else {
            { count, state.position, void } = biss_encoder(data, position_feedback_config);
        }
        if (position_feedback_config.polarity == SENSOR_POLARITY_INVERTED) {
            count = -count;
            state.position = position_feedback_config.resolution - state.position - 1;
        }
        if (position_feedback_config.biss_config.multiturn_resolution != 0) {
            state.count = count;
        } else {
            multiturn(state.count, state.last_position, state.position, position_feedback_config.resolution);
        }
        if (position_feedback_config.biss_config.singleturn_resolution > 12) {
            state.angle = (position_feedback_config.pole_pairs * (state.position >> (position_feedback_config.biss_config.singleturn_resolution-12))) & 4095;
        } else {
            state.angle = (position_feedback_config.pole_pairs * (state.position << (12-position_feedback_config.biss_config.singleturn_resolution))) & 4095;
        }
        break;
    }
    state.last_position = state.position;
    return;
}

int init_sensor(port * biss_clock_port, port * biss_data_port, SPIPorts * spi_ports,
        int biss_clock_low, int biss_clock_high, PositionFeedbackConfig &position_feedback_config, int sensor_type, PositionState &pos_state,
        timer t, unsigned int &last_read)
{
    int read_period = 0;

    //init
    switch(sensor_type)
    {
    case REM_16MT_SENSOR:
        //init sensor
        init_spi_ports(*spi_ports);
        pos_state.status = rem_16mt_init(*spi_ports, position_feedback_config);
        if (pos_state.status != SENSOR_NO_ERROR) { //wait 200 ms and retry init
            delay_ticks(200000*position_feedback_config.ifm_usec);
            pos_state.status = rem_16mt_init(*spi_ports, position_feedback_config);
        }
        read_period = position_feedback_config.ifm_usec*REM_16MT_POLLING_TIME;
        break;
    case REM_14_SENSOR:
        init_spi_ports(*spi_ports);
        pos_state.status = initRotarySensor(*spi_ports,  position_feedback_config);
        read_period = position_feedback_config.ifm_usec*REM_14_POLLING_TIME;
        break;
    case BISS_SENSOR:
    case SSI_SENSOR:
#ifdef DEBUG_POSITION_FEEDBACK
        //check if resolution is a power of 2
        if ( position_feedback_config.resolution & (position_feedback_config.resolution -1) )
        {
            printstrln("BISS service: Wrong resolution");
        }
#endif
        position_feedback_config.biss_config.singleturn_resolution = tickstobits(position_feedback_config.resolution);
        break;
    }
    //read
    for (int i=0;i<2;i++) { //read 2 times
        read_position(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);
    }


#ifdef DEBUG_POSITION_FEEDBACK
    switch(sensor_type)
    {
    case BISS_SENSOR:
        if (pos_state.status == SENSOR_CHECKSUM_ERROR)
            printstrln("biss_service: ERROR: CRC");
        else if (pos_state.status == SENSOR_BISS_NO_START_BIT_ERROR)
            printstrln("biss_service: ERROR: No Start bit");
        else if (pos_state.status == SENSOR_BISS_NO_ACK_BIT_ERROR)
            printstrln("biss_service: ERROR: No Ack bit");
        else if (pos_state.status != SENSOR_NO_ERROR)
            printstrln("biss_service: ERROR: initialization");
        printstr(start_message);
        printstrln("BISS");
        break;
    case SSI_SENSOR:
        if (pos_state.status != SENSOR_NO_ERROR) {
            printstrln("biss_service: ERROR: initialization");
        }
        printstr(start_message);
        printstrln("SSI");
        break;
    case REM_16MT_SENSOR:
        if (pos_state.status != SENSOR_NO_ERROR) {
            delay_ticks(200000*position_feedback_config.ifm_usec);
            pos_state.status = rem_16mt_init(*spi_ports, position_feedback_config);
            if (pos_state.status != SENSOR_NO_ERROR) {
                printstr("Error with REM_16MT sensor initialization");
                printintln(pos_state.status);
            }
        }
        printstr(start_message);
        printstrln("REM_16MT");
        break;
    case REM_14_SENSOR:
        if (pos_state.status != SENSOR_NO_ERROR) {
            printstrln("Error with SPI REM_14 sensor");
        }
        printstr(start_message);
        printstrln("REM_14");
        break;
    }
#endif


    return read_period;
}


void serial_encoder_service(port * biss_clock_port, port * biss_data_port, SPIPorts * spi_ports, port * (&?gpio_ports)[4],
                int biss_clock_low, int biss_clock_high, PositionFeedbackConfig &position_feedback_config,
                client interface shared_memory_interface ?i_shared_memory,
                interface PositionFeedbackInterface server i_position_feedback[3],
                int gpio_on)
{

    //init variables
    int sensor_type = position_feedback_config.sensor_type;
    SensorError last_sensor_error = SENSOR_NO_ERROR;
    const int sensor_error_limit = 100;
    int sensor_error_count = 0;
    //velocity
    int velocity = 0;
    int velocity_buffer[8] = {0};
    int index = 0;
    int old_count = 0;
    int crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
    char old_timestamp = 0, timediff;
    int timediff_long = 0;
    //position
    PositionState pos_state = {0};
    int singleturn = 0;
    //timing
    timer t;
    unsigned int last_read = 0;
    t :> last_read;
    unsigned int last_velocity_read = last_read;
    unsigned int next_read = last_read;
    unsigned int sensor_error_check_time = last_read;
    unsigned int end_time = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    int read_period = init_sensor(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);


    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                read_position(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);
                angle = pos_state.angle;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position , SensorError status }:
                read_position(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);
                out_count = pos_state.count + position_feedback_config.offset;
                position = pos_state.position;
                status = pos_state.status;
                break;

        //send velocity
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //receive new config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                last_sensor_error = SENSOR_NO_ERROR;
                sensor_error_count = 0;
                UsecType ifm_usec = position_feedback_config.ifm_usec;
                position_feedback_config = in_config;
                position_feedback_config.ifm_usec = ifm_usec;
                read_period = init_sensor(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);
                crossover = position_feedback_config.resolution - position_feedback_config.resolution/10;
                write_hall_state_angle_shared_memory(position_feedback_config, i_shared_memory);
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
                switch(sensor_type)
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
                    t when timerafter(last_read + REM_16MT_TIMEOUT*position_feedback_config.ifm_usec) :> void;
                    rem_16mt_write(*spi_ports, REM_16MT_CONF_PRESET, (multiturn << 16) + singleturn, 32, position_feedback_config.ifm_usec);
                    break;
                case REM_14_SENSOR:
                    { pos_state.last_position, pos_state.status } = readRotarySensorAngleWithoutCompensation(*spi_ports, position_feedback_config.ifm_usec);
                    break;
                case BISS_SENSOR:
                case SSI_SENSOR:
                    read_position(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);
                    break;
                }
                t :> last_read;
                pos_state.count = new_count;
                break;

        //execute command
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                if (sensor_type == REM_16MT_SENSOR)
                {
                    t when timerafter(last_read + REM_16MT_TIMEOUT*position_feedback_config.ifm_usec) :> void;
                    rem_16mt_write(*spi_ports, opcode, data, data_bits, position_feedback_config.ifm_usec);
                    { status, void, void, void, void } = rem_16mt_read(*spi_ports, position_feedback_config.ifm_usec);
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
        case t when timerafter(next_read) :> next_read:
            read_position(biss_clock_port, biss_data_port, spi_ports, biss_clock_low, biss_clock_high, position_feedback_config, sensor_type, pos_state, t, last_read);

            //time between last velocity computation
            if (sensor_type == REM_16MT_SENSOR) {
                //timestamp difference
                timediff = (char)pos_state.timestamp-old_timestamp;
                timediff_long += timediff;
                old_timestamp = pos_state.timestamp;
            } else {
                timediff_long = (last_read-last_velocity_read)/position_feedback_config.ifm_usec;
            }

            //compute velocity every position_feedback_config.velocity_compute_period microseconds
            if (timeafter(last_read, last_velocity_read+position_feedback_config.ifm_usec*position_feedback_config.velocity_compute_period)) {
                int difference = pos_state.count - old_count;
                old_count = pos_state.count;

                if (timediff_long != 0 && difference < crossover && difference > -crossover) {
                    // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
                    //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
                    velocity = velocity_compute(difference, timediff_long, position_feedback_config.resolution);

                    //filter for REM_16MT_SENSOR
                    if (sensor_type == REM_16MT_SENSOR || sensor_type == BISS_SENSOR || sensor_type == SSI_SENSOR) {
                        velocity = filter(velocity_buffer, index, 8, velocity);
                        timediff_long = 0;
                    }
                }
                last_velocity_read = last_read;
            }


#ifdef XSCOPE_REM_16MT
            xscope_int(VELOCITY, velocity);
            xscope_int(COUNT, pos_state.count);
            xscope_int(POSITION, pos_state.position);
            xscope_int(STATUS, pos_state.status*500);
            xscope_int(TIMESTAMP, timediff);
            if (pos_state.status != SENSOR_CHECKSUM_ERROR) {
                xscope_int(CHECKSUM_ERROR, 0);
            } else {
                xscope_int(CHECKSUM_ERROR, 1000);
            }
#endif


            //store last error
            if (pos_state.status != SENSOR_NO_ERROR) {
                if (sensor_error_count < sensor_error_limit) {
                    sensor_error_count++;
                    //reset sensor error count
                    if (timeafter(last_read, sensor_error_check_time)) {
                        sensor_error_check_time = last_read + 10000*position_feedback_config.ifm_usec;//10 msec
                        sensor_error_count = 0;
                    }
                } else {
                    last_sensor_error = pos_state.status;
                }
            }

            //format the sensor singleturn position as a 16-bit data
            switch(sensor_type)
            {
            case REM_16MT_SENSOR:
                singleturn = pos_state.position;
                break;
            case REM_14_SENSOR:
                singleturn = pos_state.position << 2;
                break;
            case BISS_SENSOR:
            case SSI_SENSOR:
                if (position_feedback_config.biss_config.singleturn_resolution > 16) {
                    singleturn = pos_state.position >> (position_feedback_config.biss_config.singleturn_resolution -16);
                } else {
                    singleturn = pos_state.position << (16 - position_feedback_config.biss_config.singleturn_resolution);
                }
                break;
            }

            //send data to shared memory
            write_shared_memory(i_shared_memory, position_feedback_config.sensor_function, pos_state.count + position_feedback_config.offset, singleturn, velocity, pos_state.angle, 0, 0, pos_state.status, last_sensor_error, last_read/position_feedback_config.ifm_usec);

            //gpio
            gpio_shared_memory(gpio_ports, position_feedback_config, i_shared_memory, gpio_on);


            //compute next loop time
            if (sensor_type == BISS_SENSOR || sensor_type == SSI_SENSOR) {
                //for BiSS we read just after the timeout is finished
                next_read = last_read + (position_feedback_config.biss_config.timeout+2)*position_feedback_config.ifm_usec;
            } else {
                //for others we read at a fixed frequency
                next_read += read_period;
            }

            //to prevent blocking
            t :> end_time;
            if (timeafter(end_time, next_read)) {
                next_read = end_time + position_feedback_config.ifm_usec;
            }
            break;
        }
    }
}

