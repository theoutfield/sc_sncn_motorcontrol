/**
 * @file biss_service.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_service.h>
#include <xclib.h>
#include <xs1.h>
#include <xscope.h>

extern char start_message[];


static inline unsigned int read_biss_bit(QEIHallPort * qei_hall_port, port *data_port, int data_port_config)
{
    unsigned int bit = 0;
    if (data_port_config) {
        *data_port :> bit;
    } else {
        qei_hall_port->p_qei_hall :> bit;
        bit = (bit >> BISS_DATA_PORT_BIT)&1;
    }
    return bit;
}


SensorError read_biss_sensor_data(QEIHallPort * qei_hall_port,
        HallEncSelectPort * hall_enc_select_port, int hall_enc_select_config,
        port * (&?gpio_ports)[4], timer t,
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
    unsigned int data_length = BISS_CDS_BIT + position_feedback_config.biss_config.multiturn_resolution +  position_feedback_config.biss_config.singleturn_resolution + position_feedback_config.biss_config.filling_bits + BISS_STATUS_BITS;
    unsigned int read_limit = position_feedback_config.biss_config.busy; //maximum number of bits to read before the start bit
    unsigned int crc_length = 32 - clz(position_feedback_config.biss_config.crc_poly); //clz: number of leading 0
    unsigned int frame_length = data_length+crc_length+1;

    //set clock and data port config
    unsigned int clock_config = 0;
    port *clock_port;
    if (position_feedback_config.biss_config.clock_port_config <= BISS_CLOCK_PORT_EXT_D3) { //clock is output on a gpio port
        clock_config = 1;
        clock_port = gpio_ports[position_feedback_config.biss_config.clock_port_config];
    }
    unsigned int data_port_config = 0;
    port *data_port;
    if (position_feedback_config.biss_config.data_port_number > ENCODER_PORT_2) { //data is input on a gpio port
        data_port_config = 1;
        data_port = gpio_ports[position_feedback_config.biss_config.data_port_number-ENCODER_PORT_EXT_D0];
    }

    //wait for the data line to go high
    t :> time;
    timeout = time + position_feedback_config.biss_config.timeout*position_feedback_config.ifm_usec;
    while(bit != 1 && timeafter(timeout, time)) {
        bit = read_biss_bit(qei_hall_port, data_port, data_port_config);
        t :> time;
    }

    //test if the line went up
    if (bit != 1) {
        return SENSOR_BISS_DATA_LINE_ERROR; //error data line
    }

    //SSI sensor, no ack, start, status bits
    if (position_feedback_config.sensor_type == SSI_SENSOR) {
        read_status = 2; //force status to 2 to skip the ack and start bit detection
        read_limit = 1 + position_feedback_config.biss_config.multiturn_resolution +  position_feedback_config.biss_config.singleturn_resolution + position_feedback_config.biss_config.filling_bits + crc_length;
        data_length = read_limit - crc_length;
        //put clock low and wait for the encoder to be ready
        if (position_feedback_config.biss_config.busy) {
            if (clock_config) { //clock is output on a gpio port
                *clock_port <:0;
            } else { //clock is output on the hall_enc_select port leftmost 2 bits
                hall_enc_select_port->p_hall_enc_select <: hall_enc_select_config;
            }
            delay_ticks(position_feedback_config.biss_config.busy*position_feedback_config.ifm_usec);
        }
    }

    //read the raw data
    while (read_limit) {
        if (clock_config) { //clock is output on a gpio port
            *clock_port <:0;
            *clock_port <:1;
        } else { //clock is output on the hall_enc_select port leftmost 2 bits
            hall_enc_select_port->p_hall_enc_select <: hall_enc_select_config;
            hall_enc_select_port->p_hall_enc_select <: position_feedback_config.biss_config.clock_port_config | hall_enc_select_config;
        }
        bit = read_biss_bit(qei_hall_port, data_port, data_port_config);

        //check ack and start bits and save the data
        if (read_status == 2) { //ack and start bit received, save data
            if (bitindex == 32) { //byte full
                data[byteindex] = readbuf; //save byte
                byteindex++;                //change to next byte
                readbuf = 0;
                bitindex = 0;
            }
            readbuf = (readbuf << 1) | bit;
            bitindex++;
        } else if (read_status) { //ack received, start not received
            if (bit) {//start bit received, set status to 2
                read_status++;
                read_limit = frame_length; //now we read exactly data_length+crc_length bits
            }
        } else if (bit == 0)  {//ack bit received, set status to 1
            read_status++;
        }
        read_limit--;
    }

    //wait for the data line to go low
    t :> time;
    timeout = time + position_feedback_config.biss_config.timeout*position_feedback_config.ifm_usec;
    while(bit != 0 && timeafter(timeout, time)) {
        bit = read_biss_bit(qei_hall_port, data_port, data_port_config);
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
    unsigned int readbuf = data[0] << BISS_CDS_BIT; //discard CDS bit
    unsigned int bitindex = BISS_CDS_BIT; //discard CDS bit
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
    //poly in reverse representation:  x^0 + x^1 + x^4 is 0b1100
    unsigned int bytes = data_length/32; //number of complete bytes
    unsigned int rest = data_length % 32; //rest of bits
    // crc for first byte
    unsigned int crc = bitrev(data[0]);
    if (bytes == 0) {
        crc = crc << (32-rest); // left align if first byte is incomplete
    }
    for (int i=1; i<bytes; i++) // crc for following the complete bytes if any
        crc32(crc, bitrev(data[i]), poly);
    if (bytes && rest) { // crc for the last incomplete byte
        crc32(crc, bitrev(data[bytes]), poly);
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
