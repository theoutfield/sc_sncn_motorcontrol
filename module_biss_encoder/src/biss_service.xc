/**
 * @file biss_service.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_service.h>
#include <xclib.h>
#include <xs1.h>
#include <xscope.h>

SensorError read_biss_sensor_data(port * biss_clock_port, port * biss_data_port,
        int biss_clock_low, int biss_clock_high,
        timer t,
        PositionFeedbackConfig &position_feedback_config, unsigned int data[])
{
    unsigned int crc  =  0;
    SensorError status = SENSOR_NO_ERROR;
    unsigned int read_status = 0;
    unsigned int bit = 0;
    unsigned int time;
    unsigned int timeout;
    unsigned int readbuf = 0;
    unsigned int bitindex = 0;
    unsigned int byteindex = 0;
    unsigned int data_length = position_feedback_config.biss_config.multiturn_resolution +  position_feedback_config.biss_config.singleturn_resolution + position_feedback_config.biss_config.filling_bits + BISS_STATUS_BITS;
    unsigned int read_limit = position_feedback_config.biss_config.busy; //maximum number of bits to read before the start bit
    unsigned int crc_length = 32 - clz(position_feedback_config.biss_config.crc_poly); //clz: number of leading 0
    unsigned int frame_length = data_length+crc_length;

    //wait for the data line to go high
    t :> time;
    timeout = time + position_feedback_config.biss_config.timeout*position_feedback_config.tile_usec;
    while(bit != 1 && timeafter(timeout, time)) {
        *biss_clock_port <: biss_clock_high;
        *biss_data_port :> bit;
        bit &= 1;
        t :> time;
    }

    //test if the line went up
    if (bit != 1) {
        return SENSOR_BISS_DATA_LINE_ERROR; //error data line
    }

    //SSI sensor, no ack, start, status bits
    if (position_feedback_config.sensor_type == SSI_SENSOR) {
        read_status = 2; //force status to 2 because there is no ack and start bit
        data_length -= BISS_STATUS_BITS; //SSI does not have status bits
        read_limit = data_length + crc_length;
        //put clock low and wait for the encoder to be ready
        if (position_feedback_config.biss_config.busy) {
            *biss_clock_port <: biss_clock_low;
            delay_ticks(position_feedback_config.biss_config.busy*position_feedback_config.tile_usec);
        }
    } else {
        //BiSS sensor: wait for ack and start bits
        while (read_limit && read_status != 2) {
            *biss_clock_port <: biss_clock_low;
            *biss_clock_port <: biss_clock_high;
            *biss_data_port :> bit;
            bit &= 1;

            //check ack and start bits
            if (read_status) { //ack received, start not received
                if (bit) {//start bit received, set status to 2
                    read_status++;
                    read_limit = frame_length; //now we read exactly data_length+crc_length bits
                    break;
                }
            } else if (bit == 0)  {//ack bit received, set status to 1
                read_status++;
            }
            read_limit--;
        }
    }

    //discard first bit
    *biss_clock_port <: biss_clock_low;
    *biss_clock_port <: biss_clock_high;
    *biss_data_port :> void;

    //read the raw data
    while (read_limit) {
        *biss_clock_port <: biss_clock_low;
        *biss_clock_port <: biss_clock_high;
        *biss_data_port :> bit;
        bit &= 1;

        //save the data
        if (bitindex == 32) { //byte full
            data[byteindex] = readbuf; //save byte
            byteindex++;                //change to next byte
            readbuf = 0;
            bitindex = 0;
        }
        readbuf = (readbuf << 1) | bit;
        bitindex++;
        read_limit--;
    }

    //wait for the data line to go low
    t :> time;
    timeout = time + position_feedback_config.biss_config.timeout*position_feedback_config.tile_usec;
    while(bit != 0 && timeafter(timeout, time)) {
        *biss_data_port :> bit;
        bit &= 1;
        t :> time;
    }

    //extract the crc from the data
    if (read_status == 2) {
        //extract crc
        //we read in reverse from last bit saved to extract the crc
        for (int i=0; i<crc_length; i++) {
            if (bitindex == 0) { //byte fully read
                byteindex--;
                readbuf = data[byteindex]; //load previous byte
                bitindex = 32;
            }
            bitindex--;
            crc |= (readbuf&1) << i;
            readbuf = readbuf >> 1;
        }
        data[byteindex] = readbuf << (32-bitindex);//left align and save the last data byte

        //check crc
        if (position_feedback_config.biss_config.crc_poly && crc != biss_crc(data, data_length, position_feedback_config.biss_config.crc_poly) ) {
            status = SENSOR_CHECKSUM_ERROR;
        }
    } else if (read_status) {
        status = SENSOR_BISS_NO_START_BIT_ERROR;
    } else {
        status = SENSOR_BISS_NO_ACK_BIT_ERROR;
    }
    return status;
}


{ int, unsigned int, SensorError } biss_encoder(unsigned int data[], PositionFeedbackConfig &position_feedback_config) {
    unsigned int biss_data_length;
    unsigned int position = 0;
    SensorError status = SENSOR_NO_ERROR;
    unsigned int status_bits = 0;
    unsigned int readbuf = data[0];
    unsigned int bitindex = 0;
    unsigned int byteindex = 0;
    int count = 0;

    if (position_feedback_config.sensor_type == BISS_SENSOR) {
        biss_data_length = position_feedback_config.biss_config.multiturn_resolution +  position_feedback_config.biss_config.singleturn_resolution + position_feedback_config.biss_config.filling_bits + BISS_STATUS_BITS; //length witout CDS bit
    } else {
        biss_data_length = position_feedback_config.biss_config.multiturn_resolution + position_feedback_config.biss_config.singleturn_resolution;
        status_bits = 0b11; // set status to no error
    }

    //read data
    for (int i=0; i<biss_data_length; i++) {
        if (bitindex == 32) {
            bitindex = 0;
            byteindex++;
            readbuf = data[byteindex];
        }
        if (i < position_feedback_config.biss_config.multiturn_resolution) {
            count = (count << 1) | ((readbuf & 0x80000000) >> 31);
        } else if (i < position_feedback_config.biss_config.multiturn_resolution + position_feedback_config.biss_config.singleturn_resolution) {
            position = (position << 1) | ((readbuf & 0x80000000) >> 31);
        } else if (i >= position_feedback_config.biss_config.multiturn_resolution + position_feedback_config.biss_config.singleturn_resolution + position_feedback_config.biss_config.filling_bits) {
            status_bits = (status_bits << 1) | ((readbuf & 0x80000000) >> 31);
        }
        readbuf = readbuf << 1;
        bitindex++;
    }

    switch(status_bits&0b11)
    {
    case 0b10:
        status = SENSOR_BISS_WARNING_BIT_ERROR;
        break;
    case 0b01:
        status = SENSOR_BISS_ERROR_BIT_ERROR;
        break;
    case 0b00:
        status = SENSOR_BISS_ERROR_AND_WARNING_BIT_ERROR;
        break;
    }
    count &= ~(~0U <<  position_feedback_config.biss_config.multiturn_resolution);
    position &= ~(~0U <<  position_feedback_config.biss_config.singleturn_resolution);
    count = (sext(count, position_feedback_config.biss_config.multiturn_resolution) * (1 << position_feedback_config.biss_config.singleturn_resolution)) + position;  //convert multiturn to signed absolute count

    return { count, position, status };
}


unsigned int biss_crc(unsigned int data[], unsigned int data_length, unsigned int poly) {
    // poly is in reverse representation with high exponent omitted:  x^0 + x^1 + x^6 is 0b110000
    unsigned int bytes = data_length/32;  // number of bytes used - 1
    unsigned int rest = data_length % 32; // number of bits in the last byte
    if (rest == 0) {
        rest = 32;
        bytes--;
    }
    // initial value is the first byte reversed and filled with (32-rest) zero bits on the right
    unsigned int crc = bitrev(data[0]) << (32-rest);
    for (int i=0; i<bytes; i++) {       // crc for the following complete bytes
        unsigned int nextbyte = (bitrev(data[i])>>rest) | (bitrev(data[i+1])<<(32-rest));
        crc32(crc, nextbyte, poly);
    }
    crc32(crc, 0, poly);                // crc for the last byte
    crc = bitrev(~crc) >> clz(poly);    // reverse and invert the crc for biss
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
