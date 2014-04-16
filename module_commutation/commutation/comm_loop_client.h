
/**
 * \file comm_loop_client.h
 * \brief Commutation Loop Client functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#pragma once

#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include <bldc_motor_config.h>

/**
 * \brief Struct for commutation parameters
 */
typedef struct S_COMMUTATION {
	int angle_variance;
	int max_speed_reached;
	int qei_forward_offset;
	int qei_backward_offset;
	int offset_forward;
	int hall_offset_clk;
	int hall_offset_cclk;
	int winding_type;
	int flag;
} commutation_par;

/**
 * \brief Initialize commutation parameters
 *
 * \param commutation_params struct defines the commutation angle parameters
 * \param hall_params struct defines the pole-pairs and gear ratio
 * \param nominal speed is the rated speed for the motor given on specs sheet
 */
void init_commutation_param(commutation_par &commutation_params, hall_par &hall_params, int nominal_speed);

/**
 * \brief Initialize commutation loop
 */
int init_commutation(chanend c_signal);

/**
 * \brief Specify sensor for motor commutation
 *
 * \channel c_commutation channel to send information to the commutation loop
 *
 *  Input
 * \param sensor_select specify sensor used for commutation through defines HALL and QEI
 */
void commutation_sensor_select(chanend c_commutation, int sensor_select);

/**
 *  \brief Set Input voltage for commutation loop
 *
 *   Output Channels
 * 	\channel c_commutation channel to send out motor voltage input value
 *
 * 	 Input
 * 	\param input_voltage is motor voltage input value to be set (range allowed -13739 to 13739)
 */
void set_commutation_sinusoidal(chanend c_commutation, int input_voltage);

/**
 *  \brief Internal function used to set the commutation parameters
 *
 *   Output Channels
 * 	\channel c_commutation channel to send out motor voltage input value
 *
 * 	 Input
 * 	\param commutation_params struct defines the commutation angle parameters
 */
void set_commutation_params(chanend c_commutation, commutation_par &commutation_params);

//
//void disable_motor(chanend c_commutation);
//
//void enable_motor(chanend c_commutation);
//
//int check_fet_state(chanend c_commutation);
