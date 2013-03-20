
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

/* initialize PI torque controller parameter*/
void init_torque_pars(torq_par &d);

/* initialize PI torque controller loop closing time parameter*/
void init_loop_pars(loop_par &d);

/* initialize PI field controller loop parameter*/
void init_field_pars(field_par &d); //optional

/* 	torque controller loop
 *
 * channel:
 *    input:  	torque input command
 *    adc:    	current sensor input
 *    c_hall_1: hall sensor input
 *    c_value:  control value out to commutation loop
 *	  sig:      signal to start adc after initialization
 *
 * */
void foc_loop(chanend sig, chanend input, chanend adc, chanend c_hall_1, chanend c_value);











