
/**
 * \file misc.xc
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <internal_config.h>

float result_sqrt;
int root_function(int arg)
{
	result_sqrt = (float) arg;
	return (int) round(sqrt(result_sqrt));
}
