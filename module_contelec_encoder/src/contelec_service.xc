/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <contelec_service.h>
#include <timer.h>
#include <print.h>
#include <xscope.h>
#include <mc_internal_constants.h>
#include <filter_blocks.h>

extern char start_message[];

static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void init_spi_ports(SPIPorts &spi_ports)
{
    spi_master_init(spi_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
    slave_deselect(spi_ports.slave_select); // Ensure slave select is in correct start state
}

void reset_spi_ports(SPIPorts &spi_ports)
{
    set_clock_on(spi_ports.spi_interface.blk2);
    set_clock_on(spi_ports.spi_interface.blk1);
    set_port_use_on(spi_ports.spi_interface.mosi);
    set_port_use_on(spi_ports.spi_interface.miso);
    set_port_use_on(spi_ports.spi_interface.sclk);
}

#ifdef CONTELEC_USE_TIMESTAMP
int checksum_compute(unsigned count, unsigned singleturn_filtered, unsigned singleturn_raw, unsigned timestamp) {
    int computed_checksum = 0x5a ^ (1 + (timestamp & 0xff)) ^ (2 + (singleturn_raw & 0xff)) ^ (3 + (singleturn_raw >> 8)) ^ (4 + (singleturn_filtered & 0xff)) ^ (5 + (singleturn_filtered >> 8)) ^ (6 + (count & 0xff)) ^ (7 + (count >> 8));
#else
int checksum_compute(unsigned count, unsigned singleturn_filtered, unsigned singleturn_raw) {
    int computed_checksum = 0x5a ^ (1 + (singleturn_raw & 0xff)) ^ (2 + (singleturn_raw >> 8)) ^ (3 + (singleturn_filtered & 0xff)) ^ (4 + (singleturn_filtered >> 8)) ^ (5 + (count & 0xff)) ^ (6 + (count >> 8));
#endif
    return computed_checksum & 0xff;
}

#ifdef CONTELEC_USE_TIMESTAMP
{ char, int, unsigned int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &spi_ports) {
    unsigned int timestamp;
#else
{ char, int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &spi_ports) {
#endif
    char status;
    int count;
    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned int checksum;
    unsigned int computed_checksum;
    unsigned int try_count = 0;
    timer t;
    unsigned last_read;
    t :> last_read;
    last_read = last_read - 40*CONTELEC_USEC - 1;

    do {
        t when timerafter(last_read + 40*CONTELEC_USEC) :> void;
        configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1); //set mosi to 1
        slave_select(spi_ports.slave_select);
        delay_ticks(10*CONTELEC_USEC); //wait for the data buffer to fill
        count = spi_master_in_short(spi_ports.spi_interface);
        singleturn_filtered = spi_master_in_short(spi_ports.spi_interface);
        singleturn_raw = spi_master_in_short(spi_ports.spi_interface);
#ifdef CONTELEC_USE_TIMESTAMP
        timestamp = spi_master_in_byte(spi_ports.spi_interface);
        checksum = spi_master_in_byte(spi_ports.spi_interface);
        slave_deselect(spi_ports.slave_select);
        t :> last_read;
        computed_checksum = checksum_compute(count, singleturn_filtered, singleturn_raw, timestamp);
#else
        checksum = spi_master_in_byte(spi_ports.spi_interface);
        slave_deselect(spi_ports.slave_select);
        t :> last_read;
        computed_checksum = checksum_compute(count, singleturn_filtered, singleturn_raw);
#endif
        try_count++;
    } while(computed_checksum != checksum && try_count <= 3);

    status = (count >> 12) + ((try_count-1) << 4);
    count = (sext(count & 0xfff, 12) * (1 << 16)) + singleturn_filtered; //convert multiturn to signed absolute count

#ifdef XSCOPE_CONTELEC
    xscope_int(CHECKSUM_ERROR, (try_count-1)*1000);
#endif

#ifdef CONTELEC_USE_TIMESTAMP
    return { status, count, singleturn_filtered, singleturn_raw, timestamp };
#else
    return { status, count, singleturn_filtered, singleturn_raw };
#endif
}


void contelec_encoder_write(SPIPorts &spi_ports, int opcode, int data, int data_bits)
{
    configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_select(spi_ports.slave_select);
    delay_ticks(100*CONTELEC_USEC);
    spi_master_out_byte(spi_ports.spi_interface, opcode);
    if (data_bits == 8) {
        spi_master_out_byte(spi_ports.spi_interface, data);
    } else if (data_bits == 16) {
        spi_master_out_short(spi_ports.spi_interface, data);
    } else if (data_bits == 32) {
        spi_master_out_word(spi_ports.spi_interface, data);
    }
    configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_deselect(spi_ports.slave_select);
    delay_ticks(200020*CONTELEC_USEC);

}

int contelec_encoder_init(SPIPorts &spi_ports, CONTELECConfig &contelec_config)
{
    int status;

    delay_ticks(100*CONTELEC_USEC);
    //reset
    contelec_encoder_write(spi_ports, CONTELEC_CTRL_RESET, 0, 0);
    //read status
#ifdef CONTELEC_USE_TIMESTAMP
    { status, void, void, void, void } = contelec_encoder_read(spi_ports);
#else
    { status, void, void, void } = contelec_encoder_read(spi_ports);
#endif
    delay_ticks(100*CONTELEC_USEC);
    if (status != 0)
        return status;
    //direction
    if (contelec_config.polarity == CONTELEC_POLARITY_INVERTED)
        contelec_encoder_write(spi_ports, CONTELEC_CONF_DIR, 1, 8);
    else
        contelec_encoder_write(spi_ports, CONTELEC_CONF_DIR, 0, 8);
    //offset
    int ticks_per_turn = (1 << contelec_config.resolution_bits);
    contelec_config.offset &= (ticks_per_turn-1);
    if (contelec_config.offset != 0) {
        int position, count, multiturn;
#ifdef CONTELEC_USE_TIMESTAMP
        { void, count, position, void, void } = contelec_encoder_read(spi_ports); //read actual position
#else
        { void, count, position, void } = contelec_encoder_read(spi_ports); //read actual position
#endif
        if (count < 0) {
            multiturn = (count / ticks_per_turn) - 1;
        } else {
            multiturn = (count / ticks_per_turn);
        }
        delay_ticks(100*CONTELEC_USEC);
        contelec_encoder_write(spi_ports, CONTELEC_CONF_PRESET, (multiturn << 16) + ((position + contelec_config.offset) & 65535), 32); //write same multiturn and single + offset
    }
    //filter
    if (contelec_config.filter == 1) {
        contelec_config.filter = 0x02;
    } else if (contelec_config.filter < 0) {
        contelec_config.filter = 0x00;
    } else if (contelec_config.filter > 9) {
        contelec_config.filter = 0x09;
    }
    contelec_encoder_write(spi_ports, CONTELEC_CONF_FILTER, contelec_config.filter, 8);
    //read status
#ifdef CONTELEC_USE_TIMESTAMP
    { status, void, void, void, void } = contelec_encoder_read(spi_ports);
#else
    { status, void, void, void } = contelec_encoder_read(spi_ports);
#endif
    delay_ticks(100*CONTELEC_USEC);
    return status;
}


 void contelec_service(SPIPorts &spi_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3])
{
    if (CONTELEC_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    init_spi_ports(spi_ports);
    int init_status = contelec_encoder_init(spi_ports, position_feedback_config.contelec_config);
    if (init_status) {
        printstr("Error with CONTELEC sensor initialization");
        printintln(init_status);
    }

    printstr(start_message);
    printstrln("CONTELEC");

    //init variables
    //velocity
    int velocity = 0;
    int velocity_buffer[8] = {0};
    int index = 0;
    int old_count = 0;
    int ticks_per_turn = (1 << position_feedback_config.contelec_config.resolution_bits);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = position_feedback_config.contelec_config.velocity_loop * CONTELEC_USEC; //velocity loop time in clock ticks
    int velocity_count = 0;
    //position
    unsigned int last_position = 0;
    int count = 0;
    //timing
    timer t;
    unsigned int next_velocity_read = 0;
    unsigned int last_read = 0;
    unsigned int last_velocity_read = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    //debug
    int actual_velocity = 0;
    int actual_count = 0;
    unsigned int actual_position = 0;
    unsigned int actual_angle = 0;
    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;
//    unsigned int period_time = 0;

    //first read
    delay_ticks(100 * CONTELEC_USEC);
#ifdef CONTELEC_USE_TIMESTAMP
    unsigned int old_timestamp = 0;
    int timediff_long = 0;
    { void, count, last_position, void, void } = contelec_encoder_read(spi_ports);
#else
    { void, count, last_position, void } = contelec_encoder_read(spi_ports);
#endif
    t :> last_read;

    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
#ifdef CONTELEC_USE_TIMESTAMP
                { void, count, last_position, angle, void } = contelec_encoder_read(spi_ports);
#else
                { void, count, last_position, angle } = contelec_encoder_read(spi_ports);
#endif
                t :> last_read;
                if (position_feedback_config.contelec_config.resolution_bits > 12)
                    angle = (position_feedback_config.contelec_config.pole_pairs * (angle >> (position_feedback_config.contelec_config.resolution_bits-12)) ) & 4095;
                else
                    angle = (position_feedback_config.contelec_config.pole_pairs * (angle << (12-position_feedback_config.contelec_config.resolution_bits)) ) & 4095;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
#ifdef CONTELEC_USE_TIMESTAMP
                { void, count, position, void, void } = contelec_encoder_read(spi_ports);
#else
                { void, count, position, void } = contelec_encoder_read(spi_ports);
#endif
                t :> last_read;
                last_position = position;
                out_count = count;
                break;

        //send position
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
                unsigned start_time;
                t :> start_time;
#ifdef CONTELEC_USE_TIMESTAMP
                { status, out_count, position, void, void } = contelec_encoder_read(spi_ports);
#else
                { status, out_count, position, void } = contelec_encoder_read(spi_ports);
#endif
                t :> last_read;
//                status = (last_read-start_time)/CONTELEC_USEC;
//                status = measurement_time;
                last_position = position;
                break;

        //send velocity
        case i_position_feedback[int i].get_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //send ticks per turn
        case i_position_feedback[int i].get_ticks_per_turn() -> unsigned int out_ticks_per_turn:
                out_ticks_per_turn = ticks_per_turn;
                break;

        //receive new contelec_config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                position_feedback_config = in_config;
                contelec_encoder_init(spi_ports, position_feedback_config.contelec_config); //init with new config
                //update variables which depend on contelec_config
                ticks_per_turn = (1 << position_feedback_config.contelec_config.resolution_bits);
                crossover = ticks_per_turn - ticks_per_turn/10;
                velocity_loop = position_feedback_config.contelec_config.velocity_loop * CONTELEC_USEC;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                t :> last_read;
                break;

        //send contelec_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_position_feedback[int i].set_position(int new_count):
                int multiturn;
                if (new_count < 0) {
                    multiturn = (new_count / ticks_per_turn) - 1;
                } else {
                    multiturn = (new_count / ticks_per_turn);
                }
                unsigned int singleturn = new_count % ticks_per_turn;
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
                contelec_encoder_write(spi_ports, CONTELEC_CONF_PRESET, (multiturn << 16) + singleturn, 32);
                last_position = singleturn;
                t :> last_read;
                count = new_count;
                break;

        //receive the new electrical angle to set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                if (position_feedback_config.contelec_config.resolution_bits > 12) {
                    new_angle = (new_angle << (position_feedback_config.contelec_config.resolution_bits-12));
                } else {
                    new_angle = (new_angle >> (12-position_feedback_config.contelec_config.resolution_bits));
                }
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
                contelec_encoder_write(spi_ports, CONTELEC_CTRL_RESET, 0, 0);//reset
                int real_position;
#ifdef CONTELEC_USE_TIMESTAMP
                { void, void, real_position, void, void } = contelec_encoder_read(spi_ports);
                delay_ticks(position_feedback_config.contelec_config.timeout);
                contelec_encoder_write(spi_ports, CONTELEC_CONF_STPRESET, new_angle / position_feedback_config.contelec_config.pole_pairs, 16);
                { void, void, out_offset, void, void } = contelec_encoder_read(spi_ports);
#else
                { void, void, real_position, void } = contelec_encoder_read(spi_ports);
                delay_ticks(position_feedback_config.contelec_config.timeout);
                contelec_encoder_write(spi_ports, CONTELEC_CONF_STPRESET, new_angle / position_feedback_config.contelec_config.pole_pairs, 16);
                { void, void, out_offset, void } = contelec_encoder_read(spi_ports);
#endif
                t :> last_read;
                out_offset = (out_offset - real_position) & (ticks_per_turn-1);
                position_feedback_config.contelec_config.offset = out_offset;
                break;

        //execute command
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
                contelec_encoder_write(spi_ports, opcode, data, data_bits);
#ifdef CONTELEC_USE_TIMESTAMP
                { status, void, void, void, void } = contelec_encoder_read(spi_ports);
#else
                { status, void, void, void } = contelec_encoder_read(spi_ports);
#endif
                t :> last_read;
                break;

        case i_position_feedback[int i].exit():
                reset_spi_ports(spi_ports);
                loop_flag = 0;
                continue;

        //compute velocity
        case t when timerafter(next_velocity_read) :> start_time:
            next_velocity_read += velocity_loop;
            int position, difference;
            unsigned int angle, status;
            t when timerafter(last_read + position_feedback_config.contelec_config.timeout) :> void;
#ifdef CONTELEC_USE_TIMESTAMP
            unsigned int timestamp;
            { status, count, position, angle, timestamp } = contelec_encoder_read(spi_ports);
#else
            { status, count, position, angle } = contelec_encoder_read(spi_ports);
#endif
            t :> last_read;
            last_position = position;

            velocity_count++;
#ifdef CONTELEC_USE_TIMESTAMP
            //timestamp difference
            char timediff = (char)timestamp-(char)old_timestamp;
            old_timestamp = timestamp;

            //velocity 8 samples (424us), average filter 8
            timediff_long += timediff;
            if (velocity_count >= 8) {
                difference = count - old_count;
                old_count = count;
                if (timediff_long != 0 && difference < crossover && difference > -crossover) {
                    velocity = (difference * (60000000/timediff_long)) / ticks_per_turn;
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
                    velocity = (difference * (60000000/((int)(last_read-last_velocity_read)/CONTELEC_USEC))) / ticks_per_turn;
                    velocity = filter(velocity_buffer, index, 8, velocity);
                }
                last_velocity_read = last_read;
                velocity_count = 0;
            }
#endif

#ifdef XSCOPE_CONTELEC
            xscope_int(VELOCITY, velocity);
            xscope_int(POSITION, count);
            xscope_int(POSITION_RAW, angle);
            xscope_int(STATUS, status*1000);
#ifdef CONTELEC_USE_TIMESTAMP
            xscope_int(TIMESTAMP, timediff);
#endif
//            xscope_int(PERIOD, (int)(last_read-period_time)/CONTELEC_USEC);
//            period_time = last_read;
#endif


            if (position_feedback_config.contelec_config.resolution_bits > 12)
                angle = (position_feedback_config.contelec_config.pole_pairs * (angle >> (position_feedback_config.contelec_config.resolution_bits-12)) ) & 4095;
            else
                angle = (position_feedback_config.contelec_config.pole_pairs * (angle << (12-position_feedback_config.contelec_config.resolution_bits)) ) & 4095;

            if (!isnull(i_shared_memory)) {
                if (position_feedback_config.contelec_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(angle, velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_angle = angle;
                    actual_position = position;
                } else if (position_feedback_config.contelec_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(angle);
                    actual_angle = angle;
                } else if (position_feedback_config.contelec_config.enable_push_service == PushPosition) {
                    i_shared_memory.write_velocity_position(velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_position = position;
                }
            }
            t :> end_time;

            measurement_time = (end_time-start_time)/CONTELEC_USEC;

            //to prevent blocking
            if (timeafter(end_time, next_velocity_read))
                next_velocity_read = end_time + CONTELEC_USEC;
            break;
        }
    }
}

