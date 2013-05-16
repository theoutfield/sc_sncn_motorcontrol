/*
 * dc_motor.c
 *
 *  Created on: May 16, 2013
 *      Author: pkanajar
 */
#include <dc_motor.h>
#include <math.h>
void coeffient_prec(coeff *d)
{
	float j, b, precision;
	int J, B;
	float b_n, b_d;
	float pi = 3.14159;

	precision = (float) d->prec;

	j = ROTOR_INERTIA * pow(2.0,precision);
	j = j/(1000* 10000); //gcm2 to kgm2 with precision

	J = (int)  round(j) ;

	b_n = (float) SPEED_TORQUE_GRADIENT_NUMERATOR;
	b_d = (float) SPEED_TORQUE_GRADIENT_DENOMINATOR;
	b = (pow(2.0,precision)*60*b_d)/ (b_n*2*pi*1000);

	B = (int)  round(b) ;
	d->J = J;
	d->B = B;
	return;
}
