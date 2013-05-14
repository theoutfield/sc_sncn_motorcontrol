
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

/**
 * \brief Torque controller loop
 *
 * \channel  input channel for setting torque input command
 * \channel  adc channel for current sensor input
 * \channel  c_hall channel for hall sensor input
 * \channel  c_value channel to send motor power output value
 * \channel  sig channel for signaling to start adc after initialization
 *
 * \param t_param sets the torque PI controller parameters
 * \param f_param sets the field PI controller parameters
 * \param l_param sets the control loop time
 */
void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall, chanend c_value, torq_par &t_param, field_par &f_param, loop_par &l_param);


void torque_ctrl_loop(chanend sig, chanend adc, chanend c_hall_1,
		chanend sync_output, chanend c_filter_current, chanend c_commutation,
		chanend c_torque);






