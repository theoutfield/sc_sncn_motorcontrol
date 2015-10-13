/*
 * biss_server.h
 *
 *  Created on: Sep 30, 2015
 *      Author: romuald
 */
#ifndef BISS_SERVER_H_
#define BISS_SERVER_H_
#include <platform.h>

#define ENC_CH1 1
#define ENC_CH2 2
#define P_BISS_DATA ENC_CH2


interface i_biss {
    { int, int, unsigned int } position();
};

typedef struct {
    int data_length;
    int multiturn_length;
    int singleturn_length;
    int status_length;
    int crc_poly;
} biss_par;

// enum for the several status informations
enum STATUS_num {
  NoError,
  CRCError,
  NoAck,
  NoStartBit,
};


void init_biss_param(biss_par &biss_params, int data_length, int multiturn_length, int singleturn_length, int status_length, unsigned int crc_poly);

void run_biss(server interface i_biss i_biss[2], port out p_biss_clk, port p_biss_data, clock clk, unsigned a, unsigned b,
              biss_par biss_params, static const int frame_bytes);

unsigned int read_biss_sensor_data(port out p_biss_clk, port p_biss_data, clock clk, unsigned int a, unsigned int b,
                                   unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes, unsigned int crc_poly);

unsigned int biss_crc(unsigned int *data, unsigned int data_length, unsigned int poly);

void biss_crc_correct(unsigned int *data, unsigned int data_length, static const unsigned int frame_bytes,
                      unsigned int crc_received, unsigned int poly);

{ int, unsigned int, unsigned int } biss_encoder(unsigned int data, int multiturn_length, int singleturn_length, int status_length);

#endif /* BISS_SERVER_H_ */
