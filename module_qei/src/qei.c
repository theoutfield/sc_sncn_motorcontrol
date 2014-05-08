
/**
 * \file qei.c
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#include <stdio.h>
#include <math.h>

extern int __qei_max_counts(int real_counts)
{
	int max_counts;
	double max_c, result;
	double max_counts_power = log10( (double)real_counts)/log10(2.0);
	max_counts = (int) ceil(max_counts_power);
	max_counts = 1 << (max_counts);
	return max_counts;
}
