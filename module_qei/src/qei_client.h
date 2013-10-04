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

#include<dc_motor_config.h>

///only position and valid
{unsigned int, unsigned int} get_qei_position(chanend c_qei, qei_par &qei_params);

//counted up position and direction
{int, int} get_qei_position_absolute(chanend c_qei);

//return velocity
int get_qei_velocity(chanend c_qei, qei_par &qei_params);

int _get_qei_velocity_pwm_resolution(chanend c_qei, qei_par &qei_params);

extern int __qei_max_counts(int real_counts);

#endif /* __QEI_CLIENT_H__ */
