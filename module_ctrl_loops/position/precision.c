#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <internal_config.h>

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
