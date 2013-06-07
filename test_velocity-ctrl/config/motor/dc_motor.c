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

	precision = (float) d->coef_prec;

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

struct S_MOTOR
{
	float J; float B;
	int var_prec;
	int coef_prec;
} mot_const;

float Kp=93000, Ki=1, Kd;
int TORQUE_INTEGRAL_MAX;
int TORQUE_OUTPUT_MAX = 13700; //PWM max

int init_constants(coeff *d)
{
	float j, b, precision, precision1;
	int J, B;
	float b_n, b_d;
	float pi = 3.14159;

	precision = (float) d->coef_prec;
	precision1 = (float) d->var_prec;

	j = ROTOR_INERTIA * pow(2.0,precision);
	j = j/(1000* 10000); //gcm2 to kgm2 with precision

	b_n = (float) SPEED_TORQUE_GRADIENT_NUMERATOR;
	b_d = (float) SPEED_TORQUE_GRADIENT_DENOMINATOR;
	b = (pow(2.0,precision)*60*b_d)/ (b_n*2*pi*1000);

	mot_const.B = b;
	mot_const.J = j;
	mot_const.coef_prec = pow(2.0,precision);
	mot_const.var_prec = pow(2.0,precision1);
	TORQUE_INTEGRAL_MAX = TORQUE_OUTPUT_MAX/Ki;
    Kd = sqrt(2*Kp);
	return 0;
}

float speed;
float acc;
float joint_torque_actual, prev_speed;
float joint_torque_set;
float error_joint_torque;
float error_joint_torque_I = 0;
float error_joint_torque_D = 0;
float error_joint_torque_previous = 0;
float joint_torque_control_out = 0;




//;

void pd_control(pd_data *p)
{

	/* computed current torque */
	speed = (float) p->speed;
	prev_speed = (float) p->prev_speed;
	joint_torque_set = (float) p->joint_torque_set;
	error_joint_torque_previous = (float) p->previous_error;

	speed = (speed*201)/60;  					//rpm to rad with prec  //get_speed_cal(c_hall_p4)
	prev_speed = (prev_speed*201)/60;
	acc = (speed - prev_speed)*333; 								// already in rad/s2 with prec

	joint_torque_actual = ((mot_const.J*acc * 1000.0)/1355.0 +( mot_const.B*speed * 1000.0)/1355.0)/mot_const.var_prec;  			// in coef_prec only
	//joint_torque_actual = joint_torque_actual ;
if(joint_torque_actual <0)
	joint_torque_actual = p->joint_torque_actual;
if(joint_torque_actual - p->joint_torque_actual > 2000)
	joint_torque_actual = p->joint_torque_actual;
	/* torque controller*/
	error_joint_torque = joint_torque_set - joint_torque_actual;

	error_joint_torque_D = (error_joint_torque - error_joint_torque_previous);

	error_joint_torque_I =  error_joint_torque_I + error_joint_torque;

	if(error_joint_torque_I > TORQUE_INTEGRAL_MAX)
	{
		error_joint_torque_I = TORQUE_INTEGRAL_MAX;
	}
	else if(error_joint_torque_I < 0)
	{
		error_joint_torque_I = 0 ;
	}

	joint_torque_control_out = Kp * error_joint_torque + Kd * error_joint_torque_D + Ki *error_joint_torque_I;

	p->joint_torque_control_out = (int) round(joint_torque_control_out);
    p->joint_torque_actual = (int) round(joint_torque_actual);

	p->error = (int) round(error_joint_torque);

	return;
}
