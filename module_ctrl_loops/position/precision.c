
/**
 * \file mot_profile.c
 *
 * Motion Profile file used to generate motion profiles for position control rountine
 * Based on Linear Function with Parabolic Blends
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define HALL 1
#define QEI 2

int position_factor(int gear_ratio, int qei_max_real, int pole_pairs, int sensor_used)
{
	double gear = (double) gear_ratio;
	double qei_max = (double) qei_max_real;
	double poles = (double) pole_pairs;
	double factor;

	if(sensor_used == QEI)
	{
		factor = (3600000.0/ (gear * qei_max))* 512.0;   //9 bit precision (QEI)
	}
	else if(sensor_used == HALL)
	{
		factor = (3600000.0 * 2.0)/ (gear * poles);      //1 bit precision (HALL)
	}
	return (int)  round(factor);
}

float result_tor;
int root_function(int arg)
{
	result_tor = (float) arg;
	//result = ;
	return (int) round(sqrt(result_tor));
}
