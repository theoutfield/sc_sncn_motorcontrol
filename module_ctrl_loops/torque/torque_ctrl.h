
/**
 * \file torque_ctrl.h
 *
 *	Torque control rountine based on field oriented Torque control method
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include <comm_loop.h>
#include <dc_motor_config.h>
#include <internal_config.h>

/**
 * \brief Get actual torque from torque control
 *
 *  Output Channel
 * \channel c_torque channel to receive actual torque
 *
 *  Output
 * \return actual torque from torque control
 */
int get_torque(chanend c_torque);

/**
 * \brief Set new target torque for torque control
 *
 *  Input Channel
 * \channel c_torque channel to signal new target torque
 *
 *  Input
 * \param torque is the new target torque
 */
void set_torque(chanend c_torque, int torque);

/**
 * \brief Torque Control Loop
 *
 *  Input
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 *
 *  Input Channel
 * \channel c_adc channel to receive to torque information from current sensor
 * \channel c_hall channel to receive position information from hall
 * \channel sync_output channel to receive synced position information from qei synced with hall
 * \channel c_torque channel to receive/send torque control information
 *
 *  Output Channel
 * \channel signal_adc channel for signaling to start adc after initialization
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void current_ctrl_loop(hall_par &hall_params, chanend signal_adc, chanend c_adc,
		chanend c_hall, chanend sync_output, chanend c_commutation,	chanend c_torque);



