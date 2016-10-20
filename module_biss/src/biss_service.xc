/**
 * @file biss_service.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_service.h>
#include <xclib.h>
#include <xs1.h>
#include <print.h>
#include <xscope.h>

#include <mc_internal_constants.h>

extern char start_message[];

static inline void update_turns(int &turns, int last_position, int position, int multiturn_resolution, int ticks_per_turn) {
    if (multiturn_resolution == 0) {
        int difference = position - last_position;
        if (difference >= ticks_per_turn/2)
            turns--;
        else if (-difference >= ticks_per_turn/2)
            turns++;
    }
}


void biss_service(QEIPorts &biss_ports, PositionFeedbackConfig &position_feedback_config, client interface shared_memory_interface ?i_shared_memory, server interface PositionFeedbackInterface i_position_feedback[3])
{

    if (BISS_USEC == USEC_FAST) { //Set freq to 250MHz
        write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz
    }

    printstr(start_message);
    printstrln("BISS");

    //init variables
    //velocity
    int velocity = 0;
    int old_count = 0;
    int old_difference = 0;
    int ticks_per_turn = (1 << position_feedback_config.biss_config.singleturn_resolution);
    int crossover = ticks_per_turn - ticks_per_turn/10;
    int velocity_loop = (position_feedback_config.biss_config.velocity_loop * BISS_USEC); //velocity loop time in clock ticks
    int velocity_factor = 60000000/position_feedback_config.biss_config.velocity_loop;
    //position
    unsigned int data[BISS_FRAME_BYTES];
    unsigned int last_position = 0;
    int last_count = 0;
    int count_offset = 0;
    int turns = 0;
    int biss_data_length = position_feedback_config.biss_config.multiturn_length +  position_feedback_config.biss_config.singleturn_length + position_feedback_config.biss_config.status_length;
    int biss_before_singleturn_length = position_feedback_config.biss_config.multiturn_length + position_feedback_config.biss_config.singleturn_length - position_feedback_config.biss_config.singleturn_resolution;
    int max_ticks_internal = (1 << (position_feedback_config.biss_config.multiturn_resolution -1 + position_feedback_config.biss_config.singleturn_resolution));
    //timing
    timer t;
    unsigned int time;
    unsigned int next_velocity_read = 0;
    unsigned int last_velocity_read = 0;
    unsigned int last_count_read = 0;
    unsigned int last_biss_read = 0;

    unsigned int measurement_time = 0;
    unsigned int start_time, end_time;

    int notification = MOTCTRL_NTF_EMPTY;

    //first read
    t :> time;
    last_biss_read = time;
    do {
        t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
        last_count = read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
        t :> last_biss_read;
    } while (last_count != NoError && !timeafter(last_biss_read, time + 1000000*BISS_USEC));
    if (last_count == CRCError)
        printstrln("biss_service: ERROR: CRC");
    else if (last_count == NoStartBit)
        printstrln("biss_service: ERROR: No Start bit");
    else if (last_count == NoAck)
        printstrln("biss_service: ERROR: No Ack bit");
    else if (last_count != NoError)
        printstrln("biss_service: ERROR: initialization");
    last_count_read = last_biss_read;
    next_velocity_read = last_biss_read;
    { last_count , last_position, void } = biss_encoder(data, position_feedback_config.biss_config);

    //main loop
    int loop_flag = 1;
    while (loop_flag) {
        select {
        //notifications
        case i_position_feedback[int i].get_notification() -> int out_notification:
                out_notification = notification;
                break;

        //send electrical angle for commutation, ajusted with electrical offset
        case i_position_feedback[int i].get_angle() -> unsigned int angle:
                t :> time;
                if (timeafter(time, last_biss_read + position_feedback_config.biss_config.timeout)) {
                    int count_internal;
                    int error = read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count_internal, angle, void } = biss_encoder(data, position_feedback_config.biss_config);
                    update_turns(turns, last_count, count_internal, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
                    last_count = count_internal;
                    last_position = angle;
                } else {
                    angle = last_position;
                }
                if (position_feedback_config.biss_config.singleturn_resolution > 12)
                    angle = (position_feedback_config.biss_config.pole_pairs * (angle >> (position_feedback_config.biss_config.singleturn_resolution-12)) + position_feedback_config.biss_config.offset_electrical ) & 4095;
                else
                    angle = (position_feedback_config.biss_config.pole_pairs * (angle << (12-position_feedback_config.biss_config.singleturn_resolution)) + position_feedback_config.biss_config.offset_electrical ) & 4095;
                if (position_feedback_config.biss_config.polarity == BISS_POLARITY_INVERTED) {
                    angle = (4096 - angle) & 4095;
                }
                break;

        //send count, position and status (error and warning bits), ajusted with count offset and polarity
        case i_position_feedback[int i].get_position() -> { int count, unsigned int position }:
                int count_internal;
                t :> time;
                if (timeafter(time, last_count_read + position_feedback_config.biss_config.timeout)) {
                    t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
                    int error = read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
                    t :> last_biss_read;
                    last_count_read = last_biss_read;
                    { count_internal, position, void } = biss_encoder(data, position_feedback_config.biss_config);
                    update_turns(turns, last_count, count_internal, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
                    last_count = count_internal;
                    last_position = position;
                } else {
                    count_internal = last_count;
                    position = last_position;
                }
                //add offset
                if (position_feedback_config.biss_config.multiturn_resolution) { //multiturn encoder
                    count = count_internal + count_offset;
                    if (count < -max_ticks_internal)
                        count = max_ticks_internal + (count % max_ticks_internal);
                    else if (count >= max_ticks_internal)
                        count = (count % max_ticks_internal) - max_ticks_internal;
                } else //singleturn encoder
                    count = turns*ticks_per_turn + count_internal  + count_offset;
                //polarity
                if (position_feedback_config.biss_config.polarity == BISS_POLARITY_INVERTED) {
                    count = -count;
                    position = ticks_per_turn - position;
                }
                //count reset
                if (count >= position_feedback_config.biss_config.max_ticks || count < -position_feedback_config.biss_config.max_ticks) {
                    count_offset = -count_internal;
                    count = 0;
                    turns = 0;
                }
                break;

        //send count, position and status (error and warning bits) as returned by the encoder (not ajusted)
        case i_position_feedback[int i].get_real_position() -> { int count, unsigned int position, unsigned int status }:
                t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
                int error = read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                { count, position, status } = biss_encoder(data, position_feedback_config.biss_config);
                status = status + (error << 2);
                update_turns(turns, last_count, count, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
                last_count = count;
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

        //receive new biss_config
        case i_position_feedback[int i].set_config(PositionFeedbackConfig in_config):
                //update variables which depend on biss_config
                position_feedback_config = in_config;
                biss_data_length = position_feedback_config.biss_config.multiturn_length +  position_feedback_config.biss_config.singleturn_length + position_feedback_config.biss_config.status_length;
                biss_before_singleturn_length = position_feedback_config.biss_config.multiturn_length + position_feedback_config.biss_config.singleturn_length - position_feedback_config.biss_config.singleturn_resolution;
                ticks_per_turn = (1 << position_feedback_config.biss_config.singleturn_resolution);
                position_feedback_config.biss_config.offset_electrical &= 4095;
                crossover = ticks_per_turn - ticks_per_turn/10;
                max_ticks_internal = (1 << (position_feedback_config.biss_config.multiturn_resolution -1 + position_feedback_config.biss_config.singleturn_resolution));
                velocity_loop = (position_feedback_config.biss_config.velocity_loop * BISS_USEC);
                velocity_factor = 60000000/position_feedback_config.biss_config.velocity_loop;

                notification = MOTCTRL_NTF_CONFIG_CHANGED;
                // TODO: Use a constant for the number of interfaces
                for (int i = 0; i < 3; i++) {
                    i_position_feedback[i].notification();
                }

                break;

        //send biss_config
        case i_position_feedback[int i].get_config() -> PositionFeedbackConfig out_config:
                out_config = position_feedback_config;
                break;

        //receive the new count to set and set the offset accordingly
        case i_position_feedback[int i].set_position(int new_count):
                t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
                read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, position;
                { count, position, void } = biss_encoder(data, position_feedback_config.biss_config);
                update_turns(turns, last_count, count, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
                last_count = count;
                last_position = position;
                if (position_feedback_config.biss_config.polarity == BISS_POLARITY_INVERTED)
                    new_count = -new_count;
                if (position_feedback_config.biss_config.multiturn_resolution == 0) {
                    turns = new_count/ticks_per_turn;
                    count_offset = new_count - ticks_per_turn*turns - count;
                } else
                    count_offset = new_count - count;
                break;

        //receive the new elecrical angle to set and set the offset accordingly
        case i_position_feedback[int i].set_angle(unsigned int new_angle) -> unsigned int offset:
                t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
                read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
                t :> last_biss_read;
                last_count_read = last_biss_read;
                int count, angle;
                { count, angle, void } = biss_encoder(data, position_feedback_config.biss_config);
                update_turns(turns, last_count, count, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
                last_count = count;
                last_position = angle;
                if (position_feedback_config.biss_config.polarity == BISS_POLARITY_INVERTED)
                    new_angle = (4096 - new_angle) & 4095;
                if (position_feedback_config.biss_config.singleturn_resolution > 12)
                    position_feedback_config.biss_config.offset_electrical = (new_angle - position_feedback_config.biss_config.pole_pairs * (angle >> (position_feedback_config.biss_config.singleturn_resolution-12)) ) & 4095;
                else
                    position_feedback_config.biss_config.offset_electrical = (new_angle - position_feedback_config.biss_config.pole_pairs * (angle >> (12-position_feedback_config.biss_config.singleturn_resolution)) ) & 4095;
                offset = position_feedback_config.biss_config.offset_electrical;
                break;

        case i_position_feedback[int i].send_command(int opcode, int data, int data_bits) -> unsigned int out_status:
                break;

        case i_position_feedback[int i].exit():
                loop_flag = 0;
                continue;

        //compute velocity
        case t when timerafter(next_velocity_read) :> start_time:
            next_velocity_read += velocity_loop;
            int count, position, angle, count_internal, difference;
            t when timerafter(last_biss_read + position_feedback_config.biss_config.timeout) :> void;
            int error = read_biss_sensor_data(biss_ports, position_feedback_config.biss_config, data, BISS_FRAME_BYTES);
            t :> last_biss_read;
//            if (error == 1) {
            last_count_read = last_biss_read;
            { count, position, void } = biss_encoder(data, position_feedback_config.biss_config);
            update_turns(turns, last_count, count, position_feedback_config.biss_config.multiturn_resolution, ticks_per_turn);
            last_count = count;
            last_position = position;
//            } else {
//                count = last_count;
//                position = last_position;
//            }

            //add offset
            if (position_feedback_config.biss_config.multiturn_resolution) { //multiturn encoder
                difference = count - old_count;
                old_count = count;
                count = count + count_offset;
                if (count < -max_ticks_internal)
                    count = max_ticks_internal + (count % max_ticks_internal);
                else if (count >= max_ticks_internal)
                    count = (count % max_ticks_internal) - max_ticks_internal;
            } else {//singleturn encoder
                count = turns*ticks_per_turn + count;
                difference = count - old_count;
                old_count = count;
                count += count_offset;
            }
//            velocity_count++;
//            if (velocity_count >= 0) {
                //check crossover
            if(difference > crossover || difference < -crossover)
                difference = old_difference;
            old_difference = difference;
            // velocity in rpm = ( difference ticks * (1 minute / velocity loop time) ) / ticks per turn
            //                 = ( difference ticks * (60,000,000 us / velocity loop time in us) ) / ticks per turn
            //            velocity = (difference * velocity_factor) / ticks_per_turn;
            velocity = (difference * (60000000/((int)(last_count_read-last_velocity_read)/BISS_USEC))) / ticks_per_turn;
            last_velocity_read = last_biss_read;
//                velocity_count = 0;
//            }

            //polarity
            if (position_feedback_config.biss_config.polarity == BISS_POLARITY_INVERTED) {
                count = -count;
                position = (ticks_per_turn - position) & (ticks_per_turn-1);
                velocity = -velocity;
            }
            if (position_feedback_config.biss_config.singleturn_resolution > 12)
                angle = (position_feedback_config.biss_config.pole_pairs * (position >> (position_feedback_config.biss_config.singleturn_resolution-12)) + position_feedback_config.biss_config.offset_electrical ) & 4095;
            else
                angle = (position_feedback_config.biss_config.pole_pairs * (position << (12-position_feedback_config.biss_config.singleturn_resolution)) + position_feedback_config.biss_config.offset_electrical ) & 4095;

            if (!isnull(i_shared_memory)) {
                if (position_feedback_config.biss_config.enable_push_service == PushAll) {
                    i_shared_memory.write_angle_velocity_position(angle, velocity, count);
                } else if (position_feedback_config.biss_config.enable_push_service == PushAngle) {
                    i_shared_memory.write_angle_electrical(angle);
                } else if (position_feedback_config.biss_config.enable_push_service == PushPosition) {
                    i_shared_memory.write_velocity_position(velocity, count);
                }
            }
            t :> end_time;

            measurement_time = (end_time-start_time)/BISS_USEC;
#ifdef XSCOPE_BISS
            xscope_int(POSITION, count);
            xscope_int(VELOCITY, velocity);
            xscope_int(ERROR, error);
            xscope_int(TIME, measurement_time);
#endif

            //to prevent blocking
            if (timeafter(end_time, next_velocity_read))
                next_velocity_read = end_time + BISS_USEC;
            break;
        }
    }
}


unsigned int read_biss_sensor_data(QEIPorts &biss_ports, BISSConfig & biss_config, unsigned int data[], static const unsigned int frame_bytes) {
    unsigned int frame[frame_bytes];
    unsigned int crc  =  0;
    unsigned int status = 0;
    unsigned int readbuf = 0;
    unsigned int bitindex = 0;
    unsigned int byteindex = 0;
    unsigned int data_length = biss_config.multiturn_length +  biss_config.singleturn_length + biss_config.status_length;
    const unsigned int timeout = 3+2; //3 bits to read before then the ack and start bits
    unsigned int crc_length = 32 - clz(biss_config.crc_poly); //clz: number of leading 0
    for (int i=0; i<(data_length-1)/32+1; i++) //init data with zeros
        data[i] = 0;

    //get the raw data
    for (int i=0; i<timeout+data_length+crc_length; i++) {
        if (bitindex == 32) {
            frame[byteindex] = readbuf;
            readbuf = 0;
            bitindex = 0;
            byteindex++;
        }
        unsigned int bit;
        biss_ports.p_qei_config <: BISS_CLK_PORT_LOW;
        biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;
        biss_ports.p_qei :> bit;
        readbuf = readbuf << 1;
        readbuf |= ((bit & (1 << BISS_DATA_PORT_BIT)) >> BISS_DATA_PORT_BIT);
        bitindex++;
    }
    biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;
    readbuf = readbuf << (31-(timeout+data_length+crc_length-1)%32); //left align the last frame byte
    frame[byteindex] = readbuf;
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
        if (biss_config.crc_poly && crc != biss_crc(data, data_length, biss_config.crc_poly) ) {
            status = CRCError;
#ifdef BISS_CRC_CORRECT
            biss_crc_correct(data, data_length, frame_bytes, crc,  biss_config.crc_poly); //try 1 bit error correction
            if (crc == biss_crc(data, data_length, biss_config.crc_poly)) {
                status = CRCCorrected;
            }
#endif
        }
    } else if (status)
        status = NoStartBit;
    else
        status = NoAck;
    return status;
}


unsigned int read_biss_sensor_data_fast(QEIPorts &biss_ports, int before_length, int data_length) {
    unsigned int data = 0;
    int status = 0;
    int timeout = 5; //3 bits to read before then the ack and start bits

    while(status < 2 && timeout > 0) {
        timeout--;
        unsigned int bit;
        biss_ports.p_qei_config <: BISS_CLK_PORT_LOW;
        biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;
        biss_ports.p_qei :> bit;
        bit = (bit & (1 << BISS_DATA_PORT_BIT));
        if (status) {
            if (bit) //status = 2, ack and start bit found
                status++;
        } else if (bit == 0) //status = 1, ack bit found
            status++;
    }
    if (timeout >= 0) {
        for (int i=0; i<before_length; i++) {
            biss_ports.p_qei_config <: BISS_CLK_PORT_LOW;
            biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;
            biss_ports.p_qei :> void;
        }
        for (int i=0; i<data_length; i++) {
            unsigned int bit;
            biss_ports.p_qei_config <: BISS_CLK_PORT_LOW;
            biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;
            biss_ports.p_qei :> bit;
            data = data << 1;
            data |= ((bit & (1 << BISS_DATA_PORT_BIT)) >> BISS_DATA_PORT_BIT);
        }
    }
    biss_ports.p_qei_config <: BISS_CLK_PORT_HIGH;

    return data;
}


{ int, unsigned int, unsigned int } biss_encoder(unsigned int data[], BISSConfig biss_config) {
    int biss_data_length = biss_config.multiturn_length +  biss_config.singleturn_length + biss_config.status_length;
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
        if (i < biss_config.multiturn_length) {
            count = count << 1;
            count |= (readbuf & 0x80000000) >> 31;
        } else if (i < biss_config.multiturn_length + biss_config.singleturn_length) {
            position = position << 1;
            position|= (readbuf & 0x80000000) >> 31;
        } else {
            status = status << 1;
            status|= (readbuf & 0x80000000) >> 31;
        }
        readbuf = readbuf << 1;
    }
    status = (~status) & ((1 << biss_config.status_length)-1);
    count &= ~(~0U <<  biss_config.multiturn_resolution);
    position &= ~(~0U <<  biss_config.singleturn_resolution);
    count = (sext(count, biss_config.multiturn_resolution) * (1 << biss_config.singleturn_resolution)) + position;  //convert multiturn to signed absolute count

    return { count, position, status };
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
