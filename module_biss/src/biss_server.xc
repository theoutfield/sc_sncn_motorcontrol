/**
 * @file biss_server.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_server.h>
#include <bldc_motor_config.h>
#include <xclib.h>
#include <xs1.h>
#include <refclk.h>
#include <stdlib.h>
#include <stdio.h>
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

static inline void update_turns(int &turns, int last_position, int position, int multiturn_resolution, int ticks_per_turn) {
    if (multiturn_resolution == 0) {
        int difference = position - last_position;
        if (difference >= ticks_per_turn/2)
            turns--;
        else if (-difference >= ticks_per_turn/2)
            turns++;
    }
}

void run_biss(server interface i_biss i_biss[n], unsigned int n, port out p_biss_clk, port in p_biss_data, clock clk,
              biss_par & biss_params, static const int frame_bytes)
{
    init_biss_param(biss_params);

    printf("*************************************\n    BISS SERVER STARTING\n*************************************\n");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << biss_params.singleturn_resolution);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = (biss_params.velocity_loop * BISS_USEC); //velocity loop time in clock ticks
    int velocity_factor = 60000000/biss_params.velocity_loop;
    //position
    unsigned int data[frame_bytes];
    unsigned int last_position = 0;
    int last_count = 0;
    int count_offset = 0;
    int turns = 0;
    int calib_flag = 0;
    int biss_data_length = biss_params.multiturn_length +  biss_params.singleturn_length + biss_params.status_length;
    int biss_before_singleturn_length = biss_params.multiturn_length + biss_params.singleturn_length - biss_params.singleturn_resolution;
    int max_ticks_internal = (1 << (biss_params.multiturn_resolution -1 + biss_params.singleturn_resolution));
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_count_read = 0;
    unsigned int last_biss_read = 0;

    //clock and port configuration
    configure_clock_rate(clk, biss_params.clock_dividend, biss_params.clock_divisor); // a/b MHz
    configure_out_port(p_biss_clk, clk, BISS_CLK_PORT_MASK);
    configure_in_port(p_biss_data, clk);

    //first read
    read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
    t :> last_biss_read;
    last_count_read = last_biss_read;
    next_velocity_read = last_biss_read;
    { last_count , last_position, void } = biss_encoder(data, biss_params);

    //main loop
    while (1) {
        [[ordered]]
        select {
        //send electrical angle for commutation, ajusted with electrical offset
        case i_biss[int i].get_angle_electrical() -> unsigned int angle:
                if (calib_flag == 0) {
                    t :> time;
                    if (timeafter(time, last_biss_read + biss_params.timeout)) {
                        angle = read_biss_sensor_data_fast(p_biss_clk, p_biss_data, clk, 0, 0, biss_before_singleturn_length, biss_params.singleturn_resolution);
                        t :> last_biss_read;
                        last_position = angle;
                    } else
                        angle = last_position;
                    if (biss_params.polarity == BISS_POLARITY_INVERTED)
                        angle = ticks_per_turn - angle;
                    if (biss_params.singleturn_resolution > 12)
                        angle = (biss_params.poles * (angle >> (biss_params.singleturn_resolution-12)) + biss_params.offset_electrical ) & 4095;
                    else
                        angle = (biss_params.poles * (angle << (12-biss_params.singleturn_resolution)) + biss_params.offset_electrical ) & 4095;
                } else
                    angle = 0;
                break;

        //send singleturn position fast
        case i_biss[int i].get_position_fast() -> unsigned int position:
                t :> time;
                if (timeafter(time, last_biss_read + biss_params.timeout)) {
                    position = read_biss_sensor_data_fast(p_biss_clk, p_biss_data, clk, 0, 0, biss_before_singleturn_length, biss_params.singleturn_resolution);
                    t :> last_biss_read;
                    last_position = position;
                } else
                    position = last_position;
                if (biss_params.polarity == BISS_POLARITY_INVERTED)
                    position = ticks_per_turn - position;
                break;

        //send count, position and status (error and warning bits), ajusted with count offset and polarity
        case i_biss[int i].get_position() -> { int count, unsigned int position, unsigned int status }:
                int count_internal;
                t :> time;
                if (timeafter(time, last_count_read + biss_params.timeout)) {
                    t when timerafter(last_biss_read + biss_params.timeout) :> void;
                    read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count_internal, position, status } = biss_encoder(data, biss_params);
                    update_turns(turns, last_count, count_internal, biss_params.multiturn_resolution, ticks_per_turn);
                    last_count = count_internal;
                    last_position = position;
                } else {
                    count_internal = last_count;
                    position = last_position;
                }
                //add offset
                if (biss_params.multiturn_resolution) { //multiturn encoder
                    count = count_internal + count_offset;
                    if (count < -max_ticks_internal)
                        count = max_ticks_internal + (count % max_ticks_internal);
                    else if (count >= max_ticks_internal)
                        count = (count % max_ticks_internal) - max_ticks_internal;
                } else //singleturn encoder
                    count = turns*ticks_per_turn + count_internal  + count_offset;
                //polarity
                if (biss_params.polarity == BISS_POLARITY_INVERTED) {
                    count = -count;
                    position = ticks_per_turn - position;
                }
                //count reset
                if (count >= biss_params.max_ticks || count < -biss_params.max_ticks) {
                    count_offset = -count_internal;
                    count = 0;
                    status = 0;
                    turns = 0;
                }
                break;

        //send count, position and status (error and warning bits) as returned by the encoder (not ajusted)
        case i_biss[int i].get_real_position() -> { int count, unsigned int position, unsigned int status }:
                t :> time;
                if (timeafter(time, last_count_read + biss_params.timeout)) {
                    t when timerafter(last_biss_read + biss_params.timeout) :> void;
                    read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count, position, status } = biss_encoder(data, biss_params);
                    update_turns(turns, last_count, count, biss_params.multiturn_resolution, ticks_per_turn);
                    last_count = count;
                    last_position = position;
                } else {
                    count = last_count;
                    position = last_position;
                    status = 0;
                }
                break;

        //send velocity
        case i_biss[int i].get_velocity() -> int velocity_:
                if (biss_params.polarity == BISS_POLARITY_NORMAL)
                    velocity_ = velocity;
                else
                    velocity_ = -velocity;
                break;

        //receive new biss_params
        case i_biss[int i].set_params(biss_par biss_params_):
                biss_params = biss_params_;
                //update variables which depend on biss_params
                configure_clock_rate(clk, biss_params.clock_dividend, biss_params.clock_divisor) ; // a/b MHz
                biss_data_length = biss_params.multiturn_length +  biss_params.singleturn_length + biss_params.status_length;
                biss_before_singleturn_length = biss_params.multiturn_length + biss_params.singleturn_length - biss_params.singleturn_resolution;
                ticks_per_turn = (1 << biss_params.singleturn_resolution);
                crossover = ticks_per_turn - ticks_per_turn/10;
                max_ticks_internal = (1 << (biss_params.multiturn_resolution -1 + biss_params.singleturn_resolution));
                velocity_loop = (biss_params.velocity_loop * BISS_USEC);
                velocity_factor = 60000000/biss_params.velocity_loop;
                break;

        //send biss_params
        case i_biss[int i].get_params() -> biss_par biss_params_:
                biss_params_ = biss_params;
                break;

        //receive the new count to set and set the offset accordingly
        case i_biss[int i].set_count(int new_count):
                t when timerafter(last_biss_read + biss_params.timeout) :> void;
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, position;
                { count, position, void } = biss_encoder(data, biss_params);
                update_turns(turns, last_count, count, biss_params.multiturn_resolution, ticks_per_turn);
                last_count = count;
                last_position = position;
                if (biss_params.polarity == BISS_POLARITY_INVERTED)
                    new_count = -new_count;
                if (biss_params.multiturn_resolution == 0) {
                    turns = new_count/ticks_per_turn;
                    count_offset = new_count - ticks_per_turn*turns - count;
                } else
                    count_offset = new_count - count;
                break;

        //receive the new elecrical angle to set and set the offset accordingly
        case i_biss[int i].set_angle_electrical(unsigned int new_angle) -> unsigned int offset:
                t when timerafter(last_biss_read + biss_params.timeout) :> void;
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_data_length, frame_bytes, biss_params.crc_poly);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, angle;
                { count, angle, void } = biss_encoder(data, biss_params);
                update_turns(turns, last_count, count, biss_params.multiturn_resolution, ticks_per_turn);
                last_count = count;
                last_position = angle;
                if (biss_params.singleturn_resolution > 12)
                    biss_params.offset_electrical = (new_angle - biss_params.poles * (angle >> (biss_params.singleturn_resolution-12)) ) & 4095;
                else
                    biss_params.offset_electrical = (new_angle - biss_params.poles * (angle >> (12-biss_params.singleturn_resolution)) ) & 4095;
                offset = biss_params.offset_electrical;
                break;

        //set the calib flag, the server will alway send 0 as electrical angle
        case i_biss[int i].set_calib(int flag):
                calib_flag = flag;
                break;

        //compute velocity
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
                update_turns(turns, last_count, count, biss_params.multiturn_resolution, ticks_per_turn);
                last_count = count;
                last_position = position;
            } else
                count = last_count;
            if (biss_params.multiturn_resolution == 0)
                count += turns*ticks_per_turn;
            int difference = count - old_count;
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_count = count;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60.000.000 us / velocity loop time in us) ) / ticks per turn
            velocity = (difference * velocity_factor) / ticks_per_turn;
            break;
        }
    }
}


unsigned int read_biss_sensor_data(port out p_biss_clk, port in p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int data[], unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly) {
    unsigned int frame[frame_bytes];
    unsigned int crc = 0;
    unsigned int status = 0;
    unsigned int bitindex = 0;
    unsigned int byteindex = 0;
    unsigned int readbuf = 0;
    const unsigned int timeout = 3+2; //3 bits to read before then the ack and start bits
    unsigned int crc_length = 32 - clz(crc_poly); //clz: number of leading 0
    for (int i=0; i<(data_length-1)/32+1; i++) //init data with zeros
        data[i] = 0;

    //clock and data port config
    if (a) { // set a to 0 to not reconfig each time
        configure_clock_rate(clk, a, b); // a/b MHz
        configure_out_port(p_biss_clk, clk, BISS_CLK_PORT_MASK);
        configure_in_port(p_biss_data, clk);
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
        p_biss_clk <: 0b0011 & BISS_CLK_PORT_MASK;
        p_biss_clk <: BISS_CLK_PORT_MASK;
        p_biss_data :> bit;
        readbuf = readbuf << 1;
        readbuf |= ((bit & (1 << BISS_DATA_PORT_BIT)) >> BISS_DATA_PORT_BIT);
        bitindex++;
    }
    stop_clock(clk);
    configure_out_port(p_biss_clk, clk, BISS_CLK_PORT_MASK);
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


unsigned int read_biss_sensor_data_fast(port out p_biss_clk, port in p_biss_data, clock clk, unsigned a, unsigned b, int before_length, int data_length) {
    unsigned int data = 0;
    int status = 0;
    int timeout = 3+2; //3 bits to read before then the ack and start bits

    //clock and data port config
    if (a) { // set a to 0 to not reconfig each time
        configure_clock_rate(clk, a, b); // a/b MHz
        configure_out_port(p_biss_clk, clk, BISS_CLK_PORT_MASK);
        configure_in_port(p_biss_data, clk);
    }

    start_clock(clk);
    while(status < 2 && timeout > 0) {
        timeout--;
        unsigned int bit;
        p_biss_clk <: 0b0011 & BISS_CLK_PORT_MASK;
        p_biss_clk <: BISS_CLK_PORT_MASK;
        p_biss_data :> bit;
        bit = (bit & (1 << BISS_DATA_PORT_BIT));
        if (status) {
            if (bit) //status = 2, ack and start bit found
                status++;
        } else if (bit == 0) //status = 1, ack bit found
            status++;
    }
    if (timeout >= 0) {
        for (int i=0; i<before_length; i++)
            p_biss_clk <: 0b0011 & BISS_CLK_PORT_MASK;
            p_biss_clk <: BISS_CLK_PORT_MASK;
            p_biss_data :> void;
        for (int i=0; i<data_length; i++) {
            unsigned int bit;
            p_biss_clk <: 0b0011 & BISS_CLK_PORT_MASK;
            p_biss_clk <: BISS_CLK_PORT_MASK;
            p_biss_data :> bit;
            data = data << 1;
            data |= ((bit & (1 << BISS_DATA_PORT_BIT)) >> BISS_DATA_PORT_BIT);
        }
    }
    stop_clock(clk);
    configure_out_port(p_biss_clk, clk, BISS_CLK_PORT_MASK);

    return data;
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
    count &= ~(~0U <<  biss_params.multiturn_resolution);
    position &= ~(~0U <<  biss_params.singleturn_resolution);
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
