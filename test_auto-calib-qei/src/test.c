

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "test.h"

float f1;

int input_torq(in_data *d )
{
	int torque, exit_mode;
	printf("enter torque\n");
	scanf("%d", &torque);
	printf("exit ? 1 0\n");
	scanf("%d", &exit_mode);
	d->set_torque = torque;
	d->exit_mode = exit_mode;
	return 1;
}

int input_shutdown(in_data *d)
{
	int shutdown;
	printf("enable 0/ disable 1\n");
	scanf("%d", &shutdown);
	d->shutdown = shutdown;
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
	int velocity, exit_mode;
	printf("enter velocity\n");
	scanf("%d", &velocity);
	printf("exit ? 1 0\n");
	scanf("%d", &exit_mode);
	d->set_velocity = velocity;
	d->exit_mode = exit_mode;
	return 1;
}

int input_mode(in_data *d)
{
	int mode;
	printf("enter mode\n");
	scanf("%d", &mode);
	d->mode = mode;
	return 1;
}

int sine_func(int arg)
{
	f1 =  (float) arg;
	return (int) sinf(f1);
}

