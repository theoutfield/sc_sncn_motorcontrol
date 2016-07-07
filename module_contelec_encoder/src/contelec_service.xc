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


static inline void slave_select(out port spi_ss)
{
    spi_ss <: 0;
}

static inline void slave_deselect(out port spi_ss)
{
    spi_ss <: 1;
}

void init_position_feedback_ports(PositionFeedbackPorts &position_feedback_ports)
{
    spi_master_init(position_feedback_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
    slave_deselect(position_feedback_ports.slave_select); // Ensure slave select is in correct start state
}

int checksum_compute(unsigned count, unsigned singleturn_filtered, unsigned singleturn_raw) {
    int computed_checksum = 0x5a ^ (1 + (singleturn_raw & 0xff)) ^ (2 + (singleturn_raw >> 8)) ^ (3 + (singleturn_filtered & 0xff)) ^ (4 + (singleturn_filtered >> 8)) ^ (5 + (count & 0xff)) ^ (6 + (count >> 8));
    return computed_checksum & 0xff;
}

{unsigned, unsigned, unsigned} checksum_correct(unsigned count, unsigned singleturn_filtered, unsigned singleturn_raw, unsigned checksum) {
    if (checksum_compute(count, singleturn_filtered, singleturn_raw) == checksum) {
        return {count, singleturn_filtered, singleturn_raw};
    }

    unsigned orig_count = count;
    unsigned orig_singleturn_filtered = singleturn_filtered;
    unsigned orig_singleturn_raw = singleturn_raw;
    unsigned mask = 1;

    for (int i=0 ; i<16 ; i++) {
        count = orig_count ^ mask;
        if (checksum_compute(count, singleturn_filtered, singleturn_raw) == checksum) {
            return {count, singleturn_filtered, singleturn_raw};
        }
        mask = mask << 1;
    }
    count = orig_count;
    mask = 1;
    for (int i=0 ; i<16 ; i++) {
        singleturn_filtered = orig_singleturn_filtered ^ mask;
        mask = mask << 1;
    }
    mask = 1;
    singleturn_filtered = orig_singleturn_filtered;
    for (int i=0 ; i<16 ; i++) {
        singleturn_raw = orig_singleturn_raw ^ mask;
        if (checksum_compute(count, singleturn_filtered, singleturn_raw) == checksum) {
            return {count, singleturn_filtered, singleturn_raw};
        }
        mask = mask << 1;
    }

    return {orig_count, orig_singleturn_filtered, orig_singleturn_raw};
}

{ char, int, unsigned int, unsigned int } contelec_encoder_read(PositionFeedbackPorts &position_feedback_ports)
{
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
        configure_out_port(position_feedback_ports.spi_interface.mosi, position_feedback_ports.spi_interface.blk2, 1); //set mosi to 1
        slave_select(position_feedback_ports.slave_select);
        delay_ticks(10*CONTELEC_USEC); //wait for the data buffer to fill
        count = spi_master_in_short(position_feedback_ports.spi_interface);
        singleturn_filtered = spi_master_in_short(position_feedback_ports.spi_interface);
        singleturn_raw = spi_master_in_short(position_feedback_ports.spi_interface);
        checksum = spi_master_in_byte(position_feedback_ports.spi_interface);
        slave_deselect(position_feedback_ports.slave_select);
        t :> last_read;
        computed_checksum = checksum_compute(count, singleturn_filtered, singleturn_raw);
        try_count++;
    } while(computed_checksum != checksum && try_count <= 3);

    status = count >> 12;
    count = (sext(count & 0xfff, 12) * (1 << 16)) + singleturn_filtered; //convert multiturn to signed absolute count

#ifdef XSCOPE_CONTELEC
    xscope_int(CHECKSUM_ERROR, try_count*1000);
#endif

    return { status, count, singleturn_filtered, singleturn_raw };
}


void contelec_encoder_write(PositionFeedbackPorts &position_feedback_ports, int opcode, int data, int data_bits)
{
    configure_out_port(position_feedback_ports.spi_interface.mosi, position_feedback_ports.spi_interface.blk2, 1);
    slave_select(position_feedback_ports.slave_select);
    delay_ticks(100*CONTELEC_USEC);
    spi_master_out_byte(position_feedback_ports.spi_interface, opcode);
    if (data_bits == 8) {
        spi_master_out_byte(position_feedback_ports.spi_interface, data);
    } else if (data_bits == 16) {
        spi_master_out_short(position_feedback_ports.spi_interface, data);
    } else if (data_bits == 32) {
        spi_master_out_word(position_feedback_ports.spi_interface, data);
    }
    configure_out_port(position_feedback_ports.spi_interface.mosi, position_feedback_ports.spi_interface.blk2, 1);
    slave_deselect(position_feedback_ports.slave_select);
    delay_ticks(200020*CONTELEC_USEC);

}

int contelec_encoder_init(PositionFeedbackPorts &position_feedback_ports, CONTELECConfig contelec_config)
{
    int status;
//    init_position_feedback_ports(position_feedback_ports);

    delay_ticks(100*CONTELEC_USEC);
    //reset
    contelec_encoder_write(position_feedback_ports, CONTELEC_CTRL_RESET, 0, 0);
    //read status
    { status, void, void, void } = contelec_encoder_read(position_feedback_ports);
    delay_ticks(100*CONTELEC_USEC);
    if (status != 0)
        return status;
    //direction
    if (contelec_config.polarity == CONTELEC_POLARITY_INVERTED)
        contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_DIR, 1, 8);
    else
        contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_DIR, 0, 8);
    //offset
    int position;
    { void, void, position, void } = contelec_encoder_read(position_feedback_ports); //read actual position
    delay_ticks(100*CONTELEC_USEC);
    contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_STPRESET, (position + contelec_config.offset) & 65535, 16); //write singleturn
    //filter
    if (contelec_config.filter == 1 || contelec_config.filter < 0 || contelec_config.filter > 9) {
        contelec_config.filter = 0x02;
    }
    contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_FILTER, contelec_config.filter, 8);
    //read status
    { status, void, void, void } = contelec_encoder_read(position_feedback_ports);
    delay_ticks(100*CONTELEC_USEC);
    return status;
}

//int check_contelec_config(CONTELECConfig &contelec_config) {
//    if(contelec_config.polarity < 0  || contelec_config.polarity > 1){
//        printstrln("Wrong CONTELEC configuration: wrong direction");
//        return ERROR;
//    }
//    if( CONTELEC_USEC <= 0 ){
//        printstrln("Wrong CONTELEC configuration: wrong CONTELEC_USEC value");
//        return ERROR;
//    }
//    if(contelec_config.timeout < 0){
//        printstrln("Wrong CONTELEC configuration: wrong timeout");
//        return ERROR;
//    }
//    if(contelec_config.pole_pairs < 1){
//        printstrln("Wrong CONTELEC configuration: wrong pole-pairs");
//        return ERROR;
//    }
//    if (contelec_config.filter == 1 || contelec_config.filter < 0 || contelec_config.filter > 9) {
//        printstrln("Wrong filter configuration");
//        contelec_config.filter = 0x02;
//    }
//    if (contelec_config.timeout < 10*CONTELEC_USEC) {
//        printstrln("Timeout time too low");
//        contelec_config.timeout = 10*CONTELEC_USEC;
//    }
//    return SUCCESS;
//}

[[combinable]]
 void contelec_service(PositionFeedbackPorts &position_feedback_ports, CONTELECConfig &contelec_config, client interface shared_memory_interface ?i_shared_memory, interface PositionFeedbackInterface server i_position_feedback[3])
{
    //Set freq to 250MHz (always needed for velocity calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

//    if(check_contelec_config(contelec_config) == ERROR){
//        printstrln("Error while checking the CONTELEC sensor configuration");
//        return;
//    }
    init_position_feedback_ports(position_feedback_ports);
    int init_status = contelec_encoder_init(position_feedback_ports, contelec_config);
    if (init_status) {
        printstr("Error with SPI CONTELEC sensor ");
        printintln(init_status);
        return;
    }

    printstr(">>   SOMANET CONTELEC SENSOR SERVICE STARTING...\n");

    //init variables
    //velocity
    int velocity = 0;
    int velocity_buffer[10] = {0};
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << contelec_config.resolution_bits);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = contelec_config.velocity_loop * CONTELEC_USEC; //velocity loop time in clock ticks
    int velocity_factor = 60000000/contelec_config.velocity_loop;
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

    int actual_velocity = 0;
    int actual_count = 0;
    unsigned int actual_position = 0;
    unsigned int actual_angle = 0;
    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;

    //first read
//    contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_MTPRESET, 0, 16); //set multiturn to 0
    delay_ticks(100 * CONTELEC_USEC);
    { void, count, last_position, void } = contelec_encoder_read(position_feedback_ports);
    t :> last_read;

    //main loop
    while (1) {
        select {
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                t when timerafter(last_read + contelec_config.timeout) :> void;
                { void, count, last_position, angle } = contelec_encoder_read(position_feedback_ports);
                t :> last_read;
                if (contelec_config.resolution_bits > 12)
                    angle = (contelec_config.pole_pairs * (angle >> (contelec_config.resolution_bits-12)) ) & 4095;
                else
                    angle = (contelec_config.pole_pairs * (angle << (12-contelec_config.resolution_bits)) ) & 4095;
                break;

        //send multiturn count and position
        case i_position_feedback[int i].get_position() -> { int out_count, unsigned int position }:
                t when timerafter(last_read + contelec_config.timeout) :> void;
                { void, count, position, void } = contelec_encoder_read(position_feedback_ports);
                t :> last_read;
                last_position = position;
                out_count = count;
                break;

        //send position
        case i_position_feedback[int i].get_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                t when timerafter(last_read + contelec_config.timeout) :> void;
                unsigned start_time;
                t :> start_time;
                { status, out_count, position, void } = contelec_encoder_read(position_feedback_ports);
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
                ticks_per_turn = (1 << in_config.contelec_config.resolution_bits);
                in_config.contelec_config.offset &= (ticks_per_turn-1);
                t when timerafter(last_read + contelec_config.timeout) :> void;
                //update variables which depend on contelec_config
                if (contelec_config.offset != in_config.contelec_config.offset) {
                    contelec_encoder_init(position_feedback_ports, in_config.contelec_config);
                } else {
                    if (contelec_config.polarity != in_config.contelec_config.polarity) {
                        contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_DIR, in_config.contelec_config.polarity, 8);
                    }
                    if (contelec_config.filter != in_config.contelec_config.filter) {
                        if (in_config.contelec_config.filter == 1 || in_config.contelec_config.filter < 0 || in_config.contelec_config.filter > 9) {
                            in_config.contelec_config.filter = 0x02;
                        }
                        contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_FILTER, in_config.contelec_config.filter, 8);
                    }
                }
                contelec_config = in_config.contelec_config;
                crossover = ticks_per_turn - ticks_per_turn/10;
                velocity_loop = contelec_config.velocity_loop * CONTELEC_USEC;
                velocity_factor = 60000000/contelec_config.velocity_loop;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                t :> last_read;
                break;

        //send contelec_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config.contelec_config = contelec_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_position_feedback[int i].set_position(int new_count):
                int multiturn = (new_count / ticks_per_turn) & 4095;
                unsigned int singleturn = new_count % ticks_per_turn;
                int test_count;
                { void, test_count } = macs(ticks_per_turn, multiturn, 0, singleturn); //convert multiturn to absolute count: ticks per turn * number of turns + position
                if (test_count != new_count) {
                    printstrln("error new count computation");
                }
                t when timerafter(last_read + contelec_config.timeout) :> void;
                contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_PRESET, (multiturn << 16) + singleturn, 32);
                last_position = singleturn;
                t :> last_read;
                count = new_count;
                break;

        //receive the new electrical angle to set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int out_offset:
                if (contelec_config.resolution_bits > 12) {
                    new_angle = (new_angle << (contelec_config.resolution_bits-12));
                } else {
                    new_angle = (new_angle >> (12-contelec_config.resolution_bits));
                }
                t when timerafter(last_read + contelec_config.timeout) :> void;
                contelec_encoder_write(position_feedback_ports, CONTELEC_CTRL_RESET, 0, 0);//reset
                int real_position;
                { void, void, real_position, void } = contelec_encoder_read(position_feedback_ports);
                delay_ticks(contelec_config.timeout);
                contelec_encoder_write(position_feedback_ports, CONTELEC_CONF_STPRESET, new_angle / contelec_config.pole_pairs, 16);
                { void, void, out_offset, void } = contelec_encoder_read(position_feedback_ports);
                t :> last_read;
                out_offset = (out_offset - real_position) & (ticks_per_turn-1);
                contelec_config.offset = out_offset;
                break;

        //execute command
        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int status:
                t when timerafter(last_read + contelec_config.timeout) :> void;
                contelec_encoder_write(position_feedback_ports, opcode, data, data_bits);
                { status, void, void, void } = contelec_encoder_read(position_feedback_ports);
                t :> last_read;
                break;

        //compute velocity
        case t when timerafter(next_velocity_read) :> start_time:
            next_velocity_read += velocity_loop;
            int position;
            unsigned int angle, status;
            t when timerafter(last_read + contelec_config.timeout) :> void;
            { status, count, position, angle } = contelec_encoder_read(position_feedback_ports);
            t :> last_read;
            last_position = position;

//            velocity_count++;
//            if (velocity_count >= 8) {
                int difference = count - old_count;
                if(difference > crossover || difference < -crossover)
                    difference = old_difference;
                old_difference = difference;
                old_count = count;
                // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
                //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
                //            velocity = (difference * velocity_factor) / ticks_per_turn;
                velocity_buffer[0] = (difference * (60000000/((int)(last_read-last_velocity_read)/CONTELEC_USEC))) / ticks_per_turn;
                velocity = 0;
                for(int ii=0; ii<10; ii++)
                    velocity += velocity_buffer[ii];
                velocity = velocity/10;
                for(int ii=9; ii>0; ii--)
                    velocity_buffer[ii] = velocity_buffer[ii-1];
                last_velocity_read = last_read;
                velocity_count = 0;
//            }

#ifdef XSCOPE_CONTELEC
        xscope_int(VELOCITY, velocity);
        xscope_int(POSITION, position);
        xscope_int(POSITION_RAW, angle);
        xscope_int(STATUS, status*1000);
//        xscope_int(PERIOD, (int)(last_read-last_velocity_read)/CONTELEC_USEC);
#endif


            if (contelec_config.resolution_bits > 12)
                angle = (contelec_config.pole_pairs * (angle >> (contelec_config.resolution_bits-12)) ) & 4095;
            else
                angle = (contelec_config.pole_pairs * (angle << (12-contelec_config.resolution_bits)) ) & 4095;

            if (!isnull(i_shared_memory)) {
                if (contelec_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(angle, velocity, count);
                    actual_count = count;
                    actual_velocity = velocity;
                    actual_angle = angle;
                    actual_position = position;
                } else if (contelec_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(angle);
                    actual_angle = angle;
                } else if (contelec_config.enable_push_service == PushPosition) {
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

