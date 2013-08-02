
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

int get_torque(chanend c_torque);

void set_torque(chanend c_torque, int torque);

void current_ctrl_loop(hall_par &hall_params, chanend signal_adc, chanend c_adc,
		chanend c_hall, chanend sync_output, chanend c_commutation,	chanend c_torque);






