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
#include "test.h"


int input_cmd(in_data *d )
{
	int torque;
	printf("enter torque\n");
	scanf("%d", &torque);
	d->set_torque = torque;
	return 1;
}

