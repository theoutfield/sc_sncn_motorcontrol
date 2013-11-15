
/**
 * \file qei_client.h
 *
 *	QEI Sensor Client Functions
 *
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and Synapticon GmbH.
 *
 * Copyright 2013, Synapticon GmbH & XMOS Ltd. All rights reserved.
 * Authors:  Martin Schwarz <mschwarz@synapticon.com> &  Ludwig Orgler <orgler@tin.it>
 *
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

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
 * \return  counted up position from qei sensor (incorporates gear ratio)
 * 			in the range [0 - encoder_resolution] * gear-ratio
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


#endif /* __QEI_CLIENT_H__ */
