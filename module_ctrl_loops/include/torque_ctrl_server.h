
/**
 * \file  torque_ctrl_server.h
 * \brief Torque Control Loop Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#include "comm_loop_client.h"
#include <bldc_motor_config.h>
#include <internal_config.h>


/**
 * \brief Torque Control Loop
 *
 *  Input
 * \param torque_ctrl_params struct defines the torque control parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_select specify the sensor to use via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_adc channel to receive to torque information from current sensor
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_torque channel to receive/send torque control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */

void torque_control(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		int sensor_used, chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl);


