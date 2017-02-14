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


unsigned int read_biss_sensor_data(QEIHallPort * qei_hall_port_1, QEIHallPort * qei_hall_port_2, HallEncSelectPort * hall_enc_select_port, int hall_enc_select_config, port * biss_clock_port, BISSConfig & biss_config, unsigned int data[], static const unsigned int frame_bytes)
{
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
    int clock_config = 0;
    if (biss_config.clock_port_config <= BISS_CLOCK_PORT_EXT_D3) { //clock is output on a gpio port
        clock_config = 1;
    }
    for (int i=0; i<timeout+data_length+crc_length; i++) {
        if (bitindex == 32) {
            frame[byteindex] = readbuf;
            readbuf = 0;
            bitindex = 0;
            byteindex++;
        }
        unsigned int bit;
        if (clock_config) { //clock is output on a gpio port
            *biss_clock_port <:0;
            *biss_clock_port <:1;
        } else { //clock is output on the hall_enc_select port leftmost 2 bits
            hall_enc_select_port->p_hall_enc_select <: hall_enc_select_config;
            hall_enc_select_port->p_hall_enc_select <: biss_config.clock_port_config | hall_enc_select_config;
        }
        if (biss_config.data_port_config == BISS_DATA_PORT_2) {
            qei_hall_port_2->p_qei_hall :> bit;
        } else {
            qei_hall_port_1->p_qei_hall :> bit;
        }
        readbuf = readbuf << 1;
        readbuf |= ((bit & (1 << BISS_DATA_PORT_BIT)) >> BISS_DATA_PORT_BIT);
        bitindex++;
    }
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
