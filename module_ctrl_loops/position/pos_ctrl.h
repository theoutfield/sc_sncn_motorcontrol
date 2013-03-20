
/**
 * \file pos_ctrl.h
 *
 *	Position control rountine based on Computed Torque control method
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



/*
 * Move command for position control
 *
 * input params
 * 		cur_p 			: start position 	 (current position)
 * 		d_pos 			: final position 	 (desired position)
 * 		vi    			: velocity
 *
 * 		channel type varibles
 * 		pos_data		: communication channel to receive position data from the encoders/hall sensors
 *
 * output params
 * 		channel type varibles
 * 		c_commutation 	: communication channel to send out voltage to the PWM routine to run the motor
 *
 */

int move(int cur_p,int d_pos, int vi, chanend c_commutation, chanend pos_data);
