/*
 * dc_motor.h
 *
 *  Created on: May 16, 2013
 *      Author: pkanajar
 */

#ifndef DC_MOTOR_H_
#define DC_MOTOR_H_

typedef struct S_COEFF{
	int J;
	int B;
	int prec;
}coeff;
#define ROTOR_INERTIA 135 						// gcm2

#define SPEED_TORQUE_GRADIENT_NUMERATOR 88	   	// rpm/mNm
#define SPEED_TORQUE_GRADIENT_DENOMINATOR 10

#define TORQUE_CONSTANT_NUMERATOR 335 			// mNm/A
#define TORQUE_CONSTANT_DENOMINATOR 10

#ifdef __XC__
void coeffient_prec(coeff &d);
#endif
#endif /* DC_MOTOR_H_ */
