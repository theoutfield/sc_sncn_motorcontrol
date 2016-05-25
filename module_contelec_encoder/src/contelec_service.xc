/*
 * rotary_sensor_new.xc
 *
 *  Created on: 26.01.2016
 *      Author: hstroetgen
 */

#include <xs1.h>
#include <contelec_service.h>
#include <stdio.h>
#include <stdlib.h>
#include <timer.h>
#include <print.h>
#include <mc_internal_constants.h>


static char rotarySensorInitialized = 0;

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
    if (rotarySensorInitialized != 1){

        spi_master_init(spi_ports.spi_interface, DEFAULT_SPI_CLOCK_DIV);
        slave_deselect(spi_ports.slave_select); // Ensure slave select is in correct start state
        rotarySensorInitialized = 1;
    }
}

{ char, int, unsigned int, unsigned int } contelec_encoder_read(SPIPorts &spi_ports)
{
    char status;
    int count;
    unsigned int singleturn_filtered;
    unsigned int singleturn_raw;
    unsigned int checksum;

    configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1); //set mosi to 1
    slave_select(spi_ports.slave_select);
    delay_ticks(10*USEC_FAST); //wait for the data buffer to fill
    count = spi_master_in_short(spi_ports.spi_interface);
    singleturn_filtered = spi_master_in_short(spi_ports.spi_interface);
    singleturn_raw = spi_master_in_short(spi_ports.spi_interface);
    checksum = spi_master_in_byte(spi_ports.spi_interface);
    slave_deselect(spi_ports.slave_select);

    status = count >> 12;
    count = sext(count & 0xfff, 12);  //convert multiturn to signed
    { void, count } = macs(1 << 16, count, 0, singleturn_filtered); //convert multiturn to absolute count: ticks per turn * number of turns + position

    return { status, count, singleturn_filtered, singleturn_raw };
}


void contelec_encoder_write(SPIPorts &spi_ports, int opcode, int data, int data_bits)
{
    configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_select(spi_ports.slave_select);
    delay_ticks(100*USEC_FAST);
    spi_master_out_byte(spi_ports.spi_interface, opcode);
    if (data_bits == 8) {
        spi_master_out_byte(spi_ports.spi_interface, data);
    } else if (data_bits == 16) {
        spi_master_out_short(spi_ports.spi_interface, data);
    }
    configure_out_port(spi_ports.spi_interface.mosi, spi_ports.spi_interface.blk2, 1);
    slave_deselect(spi_ports.slave_select);
    delay_ticks(200020*USEC_FAST);

}

int contelec_encoder_init(SPIPorts &spi_ports, CONTELECConfig contelec_config)
{
    int status;
    init_spi_ports(spi_ports);

    //reset
    contelec_encoder_write(spi_ports, 0x00, 0, 0);
    //read status
    { status, void, void, void } = contelec_encoder_read(spi_ports);
    if (status != 0)
        return status;
    //direction
    if (contelec_config.polarity == CONTELEC_POLARITY_INVERTED)
        contelec_encoder_write(spi_ports, 0x55, 0x01, 8);
    else
        contelec_encoder_write(spi_ports, 0x55, 0x00, 8);
    //offset
    int position;
    { void, void, position, void } = contelec_encoder_read(spi_ports); //read actual position
    contelec_encoder_write(spi_ports, 0x50, (position + contelec_config.offset) & 65535, 16); //write singleturn
    //filter
    if (contelec_config.filter == 1 || contelec_config.filter < 0 || contelec_config.filter > 9) {
        contelec_config.filter = 0x02;
    }
    contelec_encoder_write(spi_ports, 0x5B, contelec_config.filter, 8);
    //read status
    { status, void, void, void } = contelec_encoder_read(spi_ports);
    return status;
}

int check_contelec_config(CONTELECConfig &contelec_config) {
    if(contelec_config.polarity < 0  || contelec_config.polarity > 1){
        printstrln("Wrong CONTELEC configuration: wrong direction");
        return ERROR;
    }
    if( CONTELEC_USEC <= 0 ){
        printstrln("Wrong CONTELEC configuration: wrong CONTELEC_USEC value");
        return ERROR;
    }
    if(contelec_config.timeout < 0){
        printstrln("Wrong CONTELEC configuration: wrong timeout");
        return ERROR;
    }
    if(contelec_config.pole_pairs < 1){
        printstrln("Wrong CONTELEC configuration: wrong pole-pairs");
        return ERROR;
    }
    if (contelec_config.filter == 1 || contelec_config.filter < 0 || contelec_config.filter > 9) {
        printstrln("Wrong filter configuration");
        contelec_config.filter = 0x02;
    }
    if (contelec_config.timeout < 10*USEC_FAST) {
        printstrln("Timeout time too low");
        contelec_config.timeout = 10*USEC_FAST;
    }
    return SUCCESS;
}

[[combinable]]
 void contelec_service(SPIPorts &spi_ports, CONTELECConfig & contelec_config, interface CONTELECInterface server i_contelec[5])
{
    //Set freq to 250MHz (always needed for velocity calculation)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    if(check_contelec_config(contelec_config) == ERROR){
        printstrln("Error while checking the CONTELEC sensor configuration");
        return;
    }
    int init_status = contelec_encoder_init(spi_ports, contelec_config);
    if (init_status) {
        printstr("Error with SPI CONTELEC sensor ");
        printintln(init_status);
        return;
    }

    printstr(">>   SOMANET CONTELEC SENSOR SERVICE STARTING...\n");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << contelec_config.resolution_bits);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = contelec_config.velocity_loop * CONTELEC_USEC; //velocity loop time in clock ticks
    int velocity_factor = 60000000/contelec_config.velocity_loop;
    //position
    unsigned int last_position = 0;
    int count = 0;
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_read = 0;

    int notification = MOTCTRL_NTF_EMPTY;

    //first read
//    contelec_encoder_write(spi_ports, 0x59, 0, 16); //set multiturn to 0
    { void, count, last_position, void } = contelec_encoder_read(spi_ports);
    t :> last_read;

    //main loop
    while (1) {
        select {
        case i_contelec[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation
        case i_contelec[int i].get_contelec_angle_velocity() -> { unsigned int angle, int out_velocity , int count, unsigned int position}:
//                t :> time;
//                if (timeafter(time, last_read + contelec_config.timeout)) {
                t when timerafter(last_read + contelec_config.timeout) :> void;
                    { void, count, angle, void } = contelec_encoder_read(spi_ports);
                    t :> last_read;
                    position = angle;
                    last_position = angle;
//                } else {
//                    angle = last_position;
//                }
                if (contelec_config.resolution_bits > 12)
                    angle = (contelec_config.pole_pairs * (angle >> (contelec_config.resolution_bits-12)) ) & 4095;
                else
                    angle = (contelec_config.pole_pairs * (angle << (12-contelec_config.resolution_bits)) ) & 4095;
                out_velocity = velocity;
                break;

        //send multiturn count and position
        case i_contelec[int i].get_contelec_position() -> { int out_count, unsigned int position }:
                t :> time;
                if (timeafter(time, last_read + contelec_config.timeout)) {
                    { void, count, position, void } = contelec_encoder_read(spi_ports);
                    t :> last_read;
                    last_position = position;
                } else
                    position = last_position;
                out_count = count;
                break;

        //send position
        case i_contelec[int i].get_contelec_real_position() -> { int out_count, unsigned int position, unsigned int status }:
                t when timerafter(last_read + contelec_config.timeout) :> void;
                unsigned start_time;
                t :> start_time;
                { status, out_count, position, void } = contelec_encoder_read(spi_ports);
                t :> last_read;
                status = (last_read-start_time)/USEC_FAST;
                last_position = position;
                break;

        //send velocity
        case i_contelec[int i].get_contelec_velocity() -> int out_velocity:
                out_velocity = velocity;
                break;

        //receive new contelec_config
        case i_contelec[int i].set_contelec_config(CONTELECConfig in_config):
                ticks_per_turn = (1 << in_config.resolution_bits);
                in_config.offset &= (ticks_per_turn-1);
                delay_ticks(10*USEC_FAST);
                //update variables which depend on contelec_config
                if (contelec_config.offset != in_config.offset) {
                    contelec_encoder_init(spi_ports, in_config);
                } else {
                    if (contelec_config.polarity != in_config.polarity) {
                        contelec_encoder_write(spi_ports, 0x55, in_config.polarity, 8);
                    }
                    if (contelec_config.filter != in_config.filter) {
                        if (in_config.filter == 1 || in_config.filter < 0 || in_config.filter > 9) {
                            in_config.filter = 0x02;
                        }
                        contelec_encoder_write(spi_ports, 0x5B, in_config.filter, 8);
                    }
                }
                contelec_config = in_config;
                crossover = ticks_per_turn - ticks_per_turn/10;
                velocity_loop = contelec_config.velocity_loop * CONTELEC_USEC;
                velocity_factor = 60000000/contelec_config.velocity_loop;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 5; i++) {
                    i_contelec[i].notification();
                }

                break;

        //send contelec_config
        case i_contelec[int i].get_contelec_config() -> CONTELECConfig out_config:
                out_config = contelec_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_contelec[int i].reset_contelec_position(int new_count):
                int multiturn = (new_count / ticks_per_turn) & 4095;
                unsigned int singleturn = new_count % ticks_per_turn;
//                int test_count;
//                { void, test_count } = macs(ticks_per_turn, multiturn, 0, singleturn); //convert multiturn to absolute count: ticks per turn * number of turns + position
//                if (test_count != new_count) {
//                    printstrln("error new count computation");
//                } else {
//                    printf("multiturn %d, singleturn %d\n", multiturn, singleturn);
                    delay_ticks(10*USEC_FAST);
                    contelec_encoder_write(spi_ports, 0x50, singleturn, 16);
                    contelec_encoder_write(spi_ports, 0x59, multiturn, 16);
//                }
                last_position = singleturn;
                t :> last_read;
                count = new_count;
                break;

        //receive the new electrical angle to set the offset accordingly
        case i_contelec[int i].reset_contelec_angle(unsigned int new_angle) -> unsigned int out_offset:
                if (contelec_config.resolution_bits > 12) {
                    new_angle = (new_angle << (contelec_config.resolution_bits-12));
                } else {
                    new_angle = (new_angle >> (12-contelec_config.resolution_bits));
                }
                delay_ticks(10*USEC_FAST);
                contelec_encoder_write(spi_ports, 0x0, 0, 0);//reset
                int real_position;
                { void, void, real_position, void } = contelec_encoder_read(spi_ports);
                delay_ticks(10*USEC_FAST);
                contelec_encoder_write(spi_ports, 0x50, new_angle / contelec_config.pole_pairs, 16);
                { void, void, out_offset, void } = contelec_encoder_read(spi_ports);
                t :> last_read;
                out_offset = (out_offset - real_position) & (ticks_per_turn-1);
                contelec_config.offset = out_offset;
                break;

        //execute command
        case i_contelec[int i].command_contelec(int opcode, int data, int data_bits) -> unsigned int status:
                delay_ticks(10*USEC_FAST);
                contelec_encoder_write(spi_ports, opcode, data, data_bits);
                { status, void, void, void } = contelec_encoder_read(spi_ports);
                t :> last_read;
                break;

        //compute velocity
//        case t when timerafter(next_velocity_read) :> void:
//            next_velocity_read += velocity_loop;
//            int position;
//            t :> time;
//            if (timeafter(time, last_read + contelec_config.timeout)) {
//                { void, count, position, void } = contelec_encoder_read(spi_ports);
//                t :> last_read;
//                last_position = position;
//            }
//            int difference = count - old_count;
//            if(difference > crossover || difference < -crossover)
//                difference = old_difference;
//            old_count = count;
//            old_difference = difference;
//            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
//            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
//            velocity = (difference * velocity_factor) / ticks_per_turn;
//            break;
        }
    }
}

