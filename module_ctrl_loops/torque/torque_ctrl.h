
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

/* 	torque controller loop
 *
 * channel:
 *    input:  	torque input command
 *    adc:    	current sensor input
 *    c_hall_1: hall sensor input
 *    c_value:  control value out to commutation loop
 *	  sig:      signal to start adc after initialization
 *
 */
void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1, chanend c_value, torq_par &t_param, field_par &f_param, loop_par &l_param);









