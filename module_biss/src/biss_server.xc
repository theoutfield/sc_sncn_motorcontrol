/**
 * @file biss_server.xc
 * @brief BiSS Encoder Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#include <biss_server.h>
#include <timer.h>
#include <xclib.h>
#include <print.h>
#include <xs1.h>


void init_biss_param(biss_par &biss_params) {
    biss_params.data_length = BISS_DATA_LENGTH;
    biss_params.multiturn_length = BISS_MULTITURN_LENGTH;
    biss_params.singleturn_length = BISS_SINGLETURN_LENGTH;
    biss_params.status_length = BISS_STATUS_LENGTH;
    biss_params.crc_poly = BISS_CRC_POLY;
}


void run_biss(server interface i_biss i_biss[2], port out p_biss_clk, port p_biss_data, clock clk, unsigned a, unsigned b,
              biss_par & biss_params, static const int frame_bytes)
{
    init_biss_param(biss_params);

    printstr("*************************************\n    BISS SERVER STARTING\n*************************************\n");

    unsigned int data[frame_bytes];

    //clock and port configuration
    configure_clock_rate(clk, a, b) ; // a/b MHz
    configure_port_clock_output(p_biss_clk, clk);
    set_port_inv(p_biss_clk);
#if(BISS_DATA_PORT == ENC_CH1)
    configure_out_port(p_biss_data, clk, 0b1000); //to configure p_biss_clk as output
#else
    configure_in_port(p_biss_data, clk);
#endif

    //main loop
    while (1) {
        unsigned int biss_timeout = 0;
        select {
        case i_biss[0].get_position() -> { int count, int position, unsigned int status }:
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_params.data_length, frame_bytes, biss_params.crc_poly);
                { count, position, status } = biss_encoder(data[0], biss_params.multiturn_length, biss_params.singleturn_length, biss_params.status_length);
                biss_timeout = 1;
                break;
        case i_biss[1].get_position() -> { int count, int position, unsigned int status }:
                read_biss_sensor_data(p_biss_clk, p_biss_data, clk, 0, 0, data, biss_params.data_length, frame_bytes, biss_params.crc_poly);
                { count, position, status } = biss_encoder(data[0], biss_params.multiturn_length, biss_params.singleturn_length, biss_params.status_length);
                biss_timeout = 1;
                break;
        }
        if (biss_timeout)
            delay_microseconds(15); // biss timeout
    }
}


unsigned int read_biss_sensor_data(port out p_biss_clk, port p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly) {
    unsigned int frame[frame_bytes];
    unsigned int crc = 0;
    unsigned int status = 0;
    unsigned int index = 0;
    unsigned int timeout = 8; //max number of bits to read before the ack bit, at least 3
    unsigned int crc_length = 32 - clz(crc_poly); //clz: number of leading 0
    for (int i=0; i<(data_length-1)/32+1; i++) //init data with zeros
        data[i] = 0;

    //clock and data port config
    if (a) { // set a to 0 to not reconfig each time
        configure_clock_rate(clk, a, b) ; // a/b MHz
        configure_port_clock_output(p_biss_clk, clk);
        set_port_inv(p_biss_clk);
#if(BISS_DATA_PORT == ENC_CH1)
        configure_out_port(p_biss_data, clk, 0b1000); //to configure p_biss_clk as output
#else
        configure_in_port(p_biss_data, clk);
#endif
    }

    //get the raw data
    start_clock(clk);
    for (int j=0; j<frame_bytes; j++) {
        unsigned int readbuf = 0;
        for (int i=0; i<32; i++) {
            unsigned int bit;
            p_biss_data :> bit;
            bit = (bit & 1);
#if(BISS_DATA_PORT == ENC_CH1)
            configure_out_port(p_biss_data, clk, 0b1000); //to reconfigure p_biss_clk as output
#endif
            readbuf = readbuf << 1;
            readbuf |= bit;
        }
        frame[j] = readbuf;
    }
    stop_clock(clk);

    //process the raw data
    //search for ack and start bit
    while (status < 2 && index <= timeout) {
        unsigned int bit = (frame[index/32] & 0x80000000);
        frame[index/32] = frame[index/32] << 1;
        index++;
        if (status) {
            if (bit) //status = 2, ack and start bit found
                status++;
        } else if (bit == 0) //status = 1, ack bit found
            status++;
    }
    //extract the data and crc
    if (status == 2) {
        for (int i=0; i<data_length; i++) {
            data[i/32] = (data[i/32] << 1);
            data[i/32] |= ( (frame[index/32] & 0x80000000) >> 31 );
            frame[index/32] = frame[index/32] << 1;
            index++;
        }
        for (int i=0; i<crc_length; i++) {
            crc = (crc << 1);
            crc |= ( (frame[index/32] & 0x80000000) >> 31 );
            frame[index/32] = frame[index/32] << 1;
            index++;
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


{ int, unsigned int, unsigned int } biss_encoder(unsigned int data, int multiturn_length, int singleturn_length, int status_length) {
    int count  = data >> (singleturn_length + status_length);                               //multiturn bits
    unsigned int position  = (data >> status_length) & (~0U >> (32-singleturn_length));     //singleturn bits
    unsigned int status = data & (~0U >> (32 - status_length));                             //status bits
    if (count >= (1 << (multiturn_length-1)))                                               //convert multiturn to signed
        count = count - (1 << multiturn_length);
    count = (1 << singleturn_length) * count + position;                                    //convert multiturn to absolute count
    return { count, position, ~status };
}


unsigned int biss_crc(unsigned int *data, unsigned int data_length, unsigned int poly) {
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


void biss_crc_correct(unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes,
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
