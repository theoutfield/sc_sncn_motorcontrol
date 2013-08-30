/*
 * qei.c
 *
 *  Created on: Aug 1, 2013
 *      Author: pkanajar
 */

#include <stdio.h>
#include <math.h>

extern int __qei_max_counts(int real_counts)
{
	int max_counts;
	double max_counts_power = log10( (double)real_counts)/log10(2.0);
	max_counts = (int) round(max_counts_power);
	max_counts = 1 << (max_counts);
	return max_counts;
}
