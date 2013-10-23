/*
 *
 * File:    qei_client.h
 *
 * Get the position from the QEI server
 *
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2013
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 */                                   
#ifndef __QEI_CLIENT_H__
#define __QEI_CLIENT_H__

#include <dc_motor_config.h>
#include "filter_blocks.h"
#include <print.h>
#include <xs1.h>
#include <stdio.h>
#include "qei_config.h"


/**
 * \channel c_qei for communicating with the QEI Server
 *
 *  Input
 * \param qei_params the struct defines sensor type and resolution parameters for qei
 *
 *  Output
 * \return  position from qei sensor
 * \return  valid : not valid - 0/ valid - 1
 */
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params);


/**
 * \channel c_qei for communicating with the QEI Server
 *
 *	Output
 * \return  counted up position from qei sensor (incorporates gear ratio)
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
 * \brief initialise struct for velocity calculation from QEI sensor
 *
 *	Input
 * \qei_velocity_params  struct is initialised
 *
 */
void init_qei_velocity_params(qei_velocity_par &qei_velocity_params);

/**
 * \brief Calculates the velocity from QEI sensor in fixed timed loop
 *
 * \channel c_qei for communicating with the QEI Server
 *
 *  Input
 * \qei_params the struct defines sensor type and resolution parameters for qei
 * \qei_velocity_params struct for velocity calculation
 *
 *  Output
 * \return  velocity from qei sensor
 */
int get_qei_velocity(chanend c_qei, qei_par &qei_params, qei_velocity_par &qei_velocity_params);

{int, int, int} get_qei_sync_position(chanend c_qei);

void set_qei_sync_offset(chanend c_qei, int offset_forward, int offset_backward);

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

//return velocity
//int get_qei_velocity(chanend c_qei, qei_par &qei_params);

#endif /* __QEI_CLIENT_H__ */
