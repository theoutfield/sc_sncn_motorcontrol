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
#include "set_cmd.h"

int input_tor_cmd( tor_data *d )
{
	int tm;
	printf("enter torq\n");
	scanf("%d",&tm);
	d->var1 = tm;
	return 1;
}

