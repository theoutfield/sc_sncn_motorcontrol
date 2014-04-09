
/**
 * \file qei_client.h
 * \brief QEI Sensor Client Functions
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

/*
 * Copyright (c) 2014, Synapticon GmbH & XMOS Ltd
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

#ifndef __QEI_CLIENT_H__
#define __QEI_CLIENT_H__

#include <bldc_motor_config.h>
#include "filter_blocks.h"
#include <print.h>
#include <xs1.h>
#include <stdio.h>
#include "qei_config.h"


/**
 * \brief Get position from QEI Server
 *
 *  Output channel
 * \channel c_qei for communicating with the QEI Server
 *
 *  Input
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 *
 *  Output
 * \return  position from qei sensor in the range [0 - log(encoder_resolution)/log(2)]
 * \return  valid for qei with index sensors: not valid - 0/ valid - 1
 */
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params);


/**
 *  \brief Get absolute position from QEI Server
 *
 *  Output channel
 * \channel c_qei for communicating with the QEI Server
 *
 *	Output
 * \return  counted up position from qei sensor (incorporates set max ticks)
 * 			in the range [ -max ticks to +max ticks]
 * \return  direction of rotation, clockwise : 1 / anti-clockwise : -1
 */
{int, int} get_qei_position_absolute(chanend c_qei);


/**
 * \brief struct definition for velocity calculation from qei sensor
 */
typedef struct QEI_VELOCITY_PARAM
{
	int previous_position;
	int old_difference;
	int filter_buffer[8];
	int index;
	int filter_length;
} qei_velocity_par;


/**
 * \brief Initialize struct for velocity calculation from QEI sensor
 *
 *	Input
 * \qei_velocity_params  struct is initialised
 *
 */
void init_qei_velocity_params(qei_velocity_par &qei_velocity_params);

/**
 * \brief Calculates the velocity from QEI sensor in 1 ms loop
 *
 *	Output channel
 * \channel c_qei for communicating with the QEI Server
 *
 *  Input
 * \qei_params the struct defines sensor type and resolution parameters for qei
 * \qei_velocity_params struct for velocity calculation
 *
 *  Output
 * \return velocity from qei sensor in rpm
 */
int get_qei_velocity(chanend c_qei, qei_par &qei_params, qei_velocity_par &qei_velocity_params);

/**
 * \brief Internal function to calculate QEI position information
 *
 *  Input
 * \real_counts qei counts per rotation
 *
 *  Output
 * \return  max position from qei sensor
 */
extern int __qei_max_counts(int real_counts);

/**
 * \brief Internal function
 */
{int, int, int} get_qei_sync_position(chanend c_qei);

/**
 * \brief Internal function
 */
void set_qei_sync_offset(chanend c_qei, int offset_forward, int offset_backward);

void reset_qei_count(chanend c_qei, int offset);

#endif /* __QEI_CLIENT_H__ */
