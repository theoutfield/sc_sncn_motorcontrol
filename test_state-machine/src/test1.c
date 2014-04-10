/*
 * write.c
 *
 *  Created on: May 2, 2012
 *      Author: pkanajar @ Synapticon
 *              30.01.2013 orgler
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "test1.h"

int input_new_state(in_data *d )
{
	int state;
	printf("enter new state\n");
	scanf("%d", &state);
	d->set_state = state;
	return 1;
}
int test_get_next_state(in_data *d)
{
	int in_state, check, ctrl_input;
	printf("enter in_state, check, ctrl_input, fault\n");
	scanf("%d %d %d %d",&d->set_state, &d->check, &d->ctrl_input, &d->fault);
	return 0;
}

int input_torq(in_data *d )
{
	int torque;
	printf("enter torque\n");
	scanf("%d", &torque);
	d->set_torque = torque;
	return 1;
}

int input_cw(in_data *d )
{
	int cw;
	int op_mode;
	printf("enter cw & op_mode\n");
	scanf("%d %d", &cw, &op_mode);
	/*printf("enter op-mode");
	scanf("%d", &op_mode);*/
	d->op_mode = op_mode;
	d->ctrl_input = cw;
	return 1;
}


int input_pos(in_data *d)
{
	int position;
	printf("enter position\n");
	scanf("%d", &position);
	d->set_position = position;
	return 1;
}
int input_vel(in_data *d)
{
	int velocity;
	printf("enter velocity\n");
	scanf("%d", &velocity);
	d->set_velocity = velocity;
	return 1;
}

