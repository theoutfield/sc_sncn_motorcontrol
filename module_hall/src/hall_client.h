/*
 * Module:  module_dsc_hall
 * File:    hall_client.h
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2010
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 */
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

{int, int, int, int, int, int, int, int, int, int} get_info_hall_input(chanend c_hall);


//NEW!

/* Get speed in rpm*/
int32_t get_hall_speed(chanend c_hall);

/* Get angle in range 0 - 4095 maps to 0 - 360 degrees*/
int32_t get_hall_angle(chanend c_hall);

//end NEW!
