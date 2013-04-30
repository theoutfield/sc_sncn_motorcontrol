/**
 * Module:  module_enc_hall
 * Ludwig Orgler orgler@tin.it synapticon 04/2013
  *
 **/
#pragma once
#include <stdint.h>

/** \brief Get position, speed and delta from a hall server
 *
 *  The client library function for a hall sensor server
 *
 *  \param c_hall the channel for communicating with the hall server
 */

{int, int, int, int} get_hall_values(chanend c_hall);

{int, int, int, int} get_encoder_values(chanend c_hall);



