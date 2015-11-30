/**
 * @file biss_server.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_server.h>
#include <timer.h>
#include <xclib.h>
#include <xs1.h>
#include <refclk.h>
#include <stdlib.h>
#include <print.h>


void init_biss_param(biss_par &biss_params) {
    biss_params.multiturn_length = BISS_MULTITURN_LENGTH;
    biss_params.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
    biss_params.singleturn_length = BISS_SINGLETURN_LENGTH;
    biss_params.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
    biss_params.status_length = BISS_STATUS_LENGTH;
    biss_params.crc_poly = BISS_CRC_POLY;
    biss_params.poles = POLE_PAIRS;
    biss_params.polarity = BISS_POLARITY;
    biss_params.clock_dividend = BISS_CLOCK_DIVIDEND;
    biss_params.clock_divisor = BISS_CLOCK_DIVISOR;
    biss_params.timeout = BISS_TIMEOUT;
    biss_params.max_ticks = BISS_MAX_TICKS;
    biss_params.velocity_loop = BISS_VELOCITY_LOOP;
    biss_params.offset_electrical = BISS_OFFSET_ELECTRICAL;
}


void run_biss(server interface i_biss i_biss[n], unsigned int n, port out p_biss_clk, port in p_biss_data, clock clk,
              biss_par & biss_params, static const int frame_bytes)
{
    init_biss_param(biss_params);

    printstr("*************************************\n    BISS SERVER STARTING\n*************************************\n");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << biss_params.singleturn_resolution);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_ticks = (ticks_per_turn >> 8); //to simplify the velocity computation
    int velocity_loop = (biss_params.velocity_loop * USEC_FAST); //velocity loop time in clock ticks
    //position
    unsigned int data[frame_bytes];
    unsigned int last_position = 0;
    int last_count = 0;
    int count_offset = 0;
    int biss_data_length = biss_params.multiturn_length +  biss_params.singleturn_length + biss_params.status_length;
    int max_ticks_internal = (1 << (biss_params.multiturn_resolution -1 + biss_params.singleturn_resolution));
    //timing
    timer t;
    unsigned int last_biss_read;
    t :> last_biss_read;
    unsigned int next_velocity_read = last_biss_read + velocity_loop;
    unsigned int last_count_read = last_biss_read;
    unsigned int time;

    //clock and port configuration
    configure_clock_rate(clk, biss_params.clock_dividend, biss_params.clock_divisor) ; // a/b MHz
    configure_port_clock_output(p_biss_clk, clk);
    set_port_inv(p_biss_clk);
#if(BISS_DATA_PORT == ENC_CH1) //FIXME use a normal 1-bit output port
    configure_out_port(p_biss_data, clk, 0b1000); //to configure p_biss_clk as output
#else
    configure_in_port(p_biss_data, clk);
#endif

    //main loop
    while (1) {
        [[ordered]]
        select {
        case i_biss[int i].get_angle_electrical() -> unsigned int angle:
                t :> time;
                if (timeafter(time, last_biss_read + biss_params.timeout)) {
                    angle = biss_position_fast(p_biss_clk, p_biss_data, clk, 0, 0, biss_params.multiturn_length, biss_params.singleturn_length);
                    t :> last_biss_read;
                    last_position = angle;
                } else
                    angle = last_position;
                if (biss_params.singleturn_resolution > 12)
                    angle = (biss_params.poles * (angle >> (biss_params.singleturn_resolution-12)) + biss_params.offset_electrical ) & 4095;
                else
                    angle = (biss_params.poles * (angle << (12-biss_params.singleturn_resolution)) + biss_params.offset_electrical ) & 4095;
                break;
        case i_biss[int i].get_position_fast() -> unsigned int position:
                t :> time;
                if (timeafter(time, last_biss_read + biss_params.timeout)) {
                    position = biss_position_fast(p_biss_clk, p_biss_data, clk, 0, 0, biss_params.multiturn_length, biss_params.singleturn_length);
                    t :> last_biss_read;
                    last_position = position;
                } else
                    position = last_position;
                break;
        case i_biss[int i].get_position() -> { int count, unsigned int position, unsigned int status }:
                int count_internal;
                t :> time;
                if (timeafter(time, last_count_read + biss_params.timeout)) {
                    t when timerafter(last_biss_read + biss_params.timeout) :> void;
                    read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count_internal, position, status } = biss_encoder(data, biss_params);
                    last_count = count_internal;
                    last_position = position;
                } else {
                    count_internal = last_count;
                    position = last_position;
                }
                count = count_internal + count_offset;
                if (count < -max_ticks_internal)
                    count = max_ticks_internal + (count % max_ticks_internal);
                else if (count >= max_ticks_internal)
                    count = (count % max_ticks_internal) - max_ticks_internal;
                if (biss_params.polarity == BISS_POLARITY_INVERTED) {
                    count = -count;
                    position -= ticks_per_turn;
                }
                if (count > biss_params.max_ticks || count < -biss_params.max_ticks) {
                    count_offset = -count_internal;
                    count = 0;
                    status = 0;
                }
                break;
        case i_biss[int i].get_real_position() -> { int count, unsigned int position, unsigned int status }:
                t :> time;
                if (timeafter(time, last_count_read + biss_params.timeout)) {
                    t when timerafter(last_biss_read + biss_params.timeout) :> void;
                    read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count, position, status } = biss_encoder(data, biss_params);
                    last_count = count;
                    last_position = position;
                } else {
                    count = last_count;
                    position = last_position;
                    status = 0;
                }
                break;
        case i_biss[int i].get_velocity() -> int velocity_:
                if (biss_params.polarity == BISS_POLARITY_NORMAL)
                    velocity_ = velocity;
                else
                    velocity_ = -velocity;
                break;
        case i_biss[int i].set_params(biss_par biss_params_):
                biss_params = biss_params_;
                configure_clock_rate(clk, biss_params.clock_dividend, biss_params.clock_divisor) ; // a/b MHz
                biss_data_length = biss_params.multiturn_length +  biss_params.singleturn_length + biss_params.status_length;
                ticks_per_turn = (1 << biss_params.singleturn_resolution);
                crossover = ticks_per_turn - ticks_per_turn/10;
                max_ticks_internal = (1 << (biss_params.multiturn_resolution -1 + biss_params.singleturn_resolution));
                velocity_ticks = (ticks_per_turn >> 8);
                velocity_loop = (biss_params.velocity_loop * USEC_FAST);
                break;
        case i_biss[int i].get_params() -> biss_par biss_params_:
                biss_params_ = biss_params;
                break;
        case i_biss[int i].set_count(int new_count):
                t when timerafter(last_biss_read + biss_params.timeout) :> void;
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, position;
                { count, position, void } = biss_encoder(data, biss_params);
                last_count = count;
                last_position = position;
                if (biss_params.polarity == BISS_POLARITY_NORMAL)
                    count_offset = new_count - count;
                else
                    count_offset = -new_count - count;
                break;
        case i_biss[int i].set_angle_electrical(unsigned int new_angle) -> unsigned int offset:
                t when timerafter(last_biss_read + biss_params.timeout) :> void;
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, angle;
                { count, angle, void } = biss_encoder(data, biss_params);
                last_count = count;
                last_position = angle;
                if (biss_params.singleturn_resolution > 12)
                    biss_params.offset_electrical = (new_angle - biss_params.poles * (angle >> (biss_params.singleturn_resolution-12)) ) & 4095;
                else
                    biss_params.offset_electrical = (new_angle - biss_params.poles * (angle >> (12-biss_params.singleturn_resolution)) ) & 4095;
                offset = biss_params.offset_electrical;
                break;
        case t when timerafter(next_velocity_read) :> void:
            next_velocity_read += velocity_loop;
            int count, position;
            t :> time;
            if (timeafter(time, last_count_read + biss_params.timeout)) {
                t when timerafter(last_biss_read + biss_params.timeout) :> void;
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                { count, position, void } = biss_encoder(data, biss_params);
                last_count = count;
                last_position = position;
            } else
                count = last_count;
            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // simplified version of: velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                                        = ( difference ticks * (60.000.000 us / 256) ) / ( (ticks per turn / 256) * velocity loop time in us)
            velocity = (difference * 234375) / (velocity_ticks * biss_params.velocity_loop); // USEC_FAST is harcoded in this
            break;
        }
    }
}


unsigned int biss_position_fast(port out p_biss_clk, port in p_biss_data, clock clk, unsigned a, unsigned b, int multiturn_length, int singleturn_length) {
    unsigned int position = 0;
    int status = 0;
    int timeout = 8;

    //clock and data port config
    if (a) { // set a to 0 to not reconfig each time
        configure_clock_rate(clk, a, b) ; // a/b MHz
        configure_port_clock_output(p_biss_clk, clk);
        set_port_inv(p_biss_clk);
#if(BISS_DATA_PORT == ENC_CH1)//FIXME use a normal 1-bit output port
        configure_out_port(p_biss_data, clk, 0b1000); //to configure p_biss_clk as output
#else
        configure_in_port(p_biss_data, clk);
#endif
    }

    start_clock(clk);
    while(status < 2 && timeout > 0) {
        timeout--;
        unsigned int bit;
        p_biss_data :> bit;
        bit = (bit & 0b10); //FIXME use a 1-bit input port
        if (status) {
            if (bit) //status = 2, ack and start bit found
                status++;
        } else if (bit == 0) //status = 1, ack bit found
            status++;
    }
    if (timeout >= 0) {
        for (int i=0; i<multiturn_length; i++)
            p_biss_data :> void;
        for (int i=0; i<singleturn_length; i++) {
            unsigned int bit;
            p_biss_data :> bit;
            position = position << 1;
            position |= ((bit & 0b10) >> 1);
        }
    }
    stop_clock(clk);

    return position;
}


unsigned int read_biss_sensor_data(port out p_biss_clk, port in p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly) {
    unsigned int frame[frame_bytes];
    unsigned int crc = 0;
    unsigned int status = 0;
    unsigned int bitindex = 0;
    unsigned int byteindex = 0;
    unsigned int readbuf = 0;
    unsigned int timeout = 8; //max number of bits to read before the ack bit, at least 3
    unsigned int crc_length = 32 - clz(crc_poly); //clz: number of leading 0
    for (int i=0; i<(data_length-1)/32+1; i++) //init data with zeros
        data[i] = 0;

    //clock and data port config
    if (a) { // set a to 0 to not reconfig each time
        configure_clock_rate(clk, a, b) ; // a/b MHz
        configure_port_clock_output(p_biss_clk, clk);
        set_port_inv(p_biss_clk);
#if(BISS_DATA_PORT == ENC_CH1)//FIXME use a normal 1-bit output port
        configure_out_port(p_biss_data, clk, 0b1000); //to configure p_biss_clk as output
#else
        configure_in_port(p_biss_data, clk);
#endif
    }

    //get the raw data
    start_clock(clk);
    for (int i=0; i<timeout+data_length+crc_length; i++) {
        if (bitindex == 32) {
            frame[byteindex] = readbuf;
            readbuf = 0;
            bitindex = 0;
            byteindex++;
        }
        unsigned int bit;
        p_biss_data :> bit;
#if(BISS_DATA_PORT == ENC_CH1)//FIXME use a normal 1-bit output port
        configure_out_port(p_biss_data, clk, 0b1000); //to reconfigure p_biss_clk as output
#endif
        readbuf = readbuf << 1;
        readbuf |= ((bit & 0b10) >> 1); //FIXME use a 1-bit input port
        bitindex++;
    }
    stop_clock(clk);
    readbuf = readbuf << (31-(timeout+data_length+crc_length-1)%32); //left align the last frame byte
    frame[byteindex] = readbuf;
    readbuf = 0;
    byteindex = 0;
    bitindex = 0;

    //process the raw data
    //search for ack and start bit
    readbuf = frame[0];
    while (status < 2 && bitindex <= timeout) {
        unsigned int bit = (readbuf & 0x80000000);
        readbuf = readbuf << 1;
        bitindex++;
        if (status) {
            if (bit) //status = 2, ack and start bit found
                status++;
        } else if (bit == 0) //status = 1, ack bit found
            status++;
    }
    //extract the data and crc
    if (status == 2) {
        for (int i=0; i<data_length; i++) {
            if (bitindex == 32) {
                bitindex = 0;
                byteindex++;
                readbuf = frame[byteindex];
            }
            data[i/32] = data[i/32] << 1;
            data[i/32] |= (readbuf & 0x80000000) >> 31;
            readbuf = readbuf << 1;
            bitindex++;
        }
        for (int i=0; i<crc_length; i++) {
            if (bitindex == 32) {
                bitindex = 0;
                byteindex++;
                readbuf = frame[byteindex];
            }
            crc = crc << 1;
            crc |= (readbuf & 0x80000000) >> 31;
            readbuf = readbuf << 1;
            bitindex++;
        }
        status = NoError;
        //check crc
        if (crc_poly && crc != biss_crc(data, data_length, crc_poly) ) {
            biss_crc_correct(data, data_length, frame_bytes, crc,  crc_poly); //try 1 bit error correction
            if (crc != biss_crc(data, data_length, crc_poly))
                status = CRCError;
        }
    } else if (status)
        status = NoStartBit;
    else
        status = NoAck;
    return status;
}


{ int, unsigned int, unsigned int } biss_encoder(unsigned int data[], biss_par biss_params) {
    int biss_data_length = biss_params.multiturn_length +  biss_params.singleturn_length + biss_params.status_length;
    unsigned int bytes = (biss_data_length-1)/32; //number of bytes used - 1
    unsigned int rest = biss_data_length % 32; //rest of bits
    int count = 0;
    unsigned int position = 0;
    unsigned int status = 0;
    unsigned int readbuf;
    int byteindex = -1;
    data[bytes] = data[bytes] << (32-rest); //left align last data byte
    for (int i=0; i<biss_data_length; i++) {
        if ((i%32) == 0) {
            byteindex++;
            readbuf = data[byteindex];
        }
        if (i < biss_params.multiturn_length) {
            count = count << 1;
            count |= (readbuf & 0x80000000) >> 31;
        } else if (i < biss_params.multiturn_length + biss_params.singleturn_length) {
            position = position << 1;
            position|= (readbuf & 0x80000000) >> 31;
        } else {
            status = status << 1;
            status|= (readbuf & 0x80000000) >> 31;
        }
        readbuf = readbuf << 1;
    }
    count = sext(count, biss_params.multiturn_resolution);  //convert multiturn to signed
    { void, count } = macs(1 << biss_params.singleturn_resolution, count, 0, position); //convert multiturn to absolute count: ticks per turn * number of turns + position
    return { count, position, ~status };
}


unsigned int biss_crc(unsigned int data[], unsigned int data_length, unsigned int poly) {
    //poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
    unsigned int bytes = data_length/32; //number of complete bytes
    unsigned int rest = data_length % 32; //rest of bits
    unsigned int datarev; //to store reversed data byte
    unsigned int crc = bitrev(data[0]);
    for (int i=1; i<bytes; i++) // compute crc for the first complete bytes
        crc32(crc, bitrev(data[i]), poly);
    if (bytes && rest) { // prepare data for the last incomplete byte
        datarev = bitrev(data[bytes]) >> (32 - rest);
        crc32(crc, datarev, poly);
        crc = crc << (32 - rest);
    }
    crc32(crc, 0, poly); //final crc
    crc = bitrev(~crc) >> clz(poly); // reverse and invert the crc for biss
    return crc;
}


void biss_crc_correct(unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int poly) {
    unsigned int crc_expected = biss_crc(data, data_length, poly);
    unsigned int crc_error = ~(crc_expected ^ crc_received) & (~0U >> clz(poly));
    unsigned int mask[frame_bytes];
    unsigned int corrected = 0;
    int i=0;
    while (i<data_length) {
        mask[i/32] = 1 << (i%32);
        if (biss_crc(mask, data_length, poly) == crc_error) {
            corrected = 1; //the bit that give the crc_error is the bit to correct
            break;
        }
        mask[i/32] = 0;
        i++;
    }
    if (corrected)
        data[i/32] = data[i/32] ^ mask[i/32];
}
