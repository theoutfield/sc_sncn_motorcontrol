
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
 * input paramters
 * \param cur_p the current position in degree
 * \param d_pos the final position (desired position) in degree
 * \param vi velocity in deg/s
 *
 * 	\channel pos_data communication channel to receive position data from the encoders/hall sensors
 *
 * output parameters
 *
 * \channel c_commutation communication channel to send motor power output value
 *
 */

int move(int cur_p,int d_pos, int vi, chanend c_commutation, chanend pos_data);
