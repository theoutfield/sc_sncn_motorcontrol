/*
#include "dc_motor.h"
#include "torque_ctrl.h"
#include <xs1.h>
#include <platform.h>
#include <print.h>
#include "ioports.h"
#include "hall_client.h"
#include "filter_blocks.h"

void torque_control(chanend c_torque, chanend c_hall_p4)
{
	/*variable declaration*
	int J;
	int B;
	int coef_prec = 15; //2^16
	int var_prec = 5;   //2^5

	coeff c;
	pd_data p;
	int speed = 0, acc = 0, prev_speed = 0;
	timer ts;
	unsigned int time, time1;
	int joint_torque_actual;
	int	joint_torque_set = 13;
	int error_joint_torque;
	int error_joint_torque_I = 0;
	int error_joint_torque_D = 0;
	int error_joint_torque_previous = 0;
	int joint_torque_control_out = 0;

	int Kp, Ki, Kd;
	int TORQUE_INTEGRAL_MAX;
	int TORQUE_OUTPUT_MAX = 13700; //PWM max
int sp1;


	#define filter_length1 1
	int filter_buffer[filter_length1];
	int index = 0, filter_output;

	init_filter(filter_buffer, index, filter_length1);



//	TORQUE_INTEGRAL_MAX = TORQUE_OUTPUT_MAX/Ki;
	/*compute motor constants from c function call*

	c.coef_prec = coef_prec;
	c.var_prec = var_prec;

	init_constants(c);


	/*ts :> time;
	p.speed = 5000;
	p.prev_speed = 5200;
	p.joint_torque_set = 13700;
	p.previous_error = 5200;
	pd_control(p);
	ts:> time1;
	printintln(time1-time); //33khz comput time*

	ts :> time;
	 //set_torque(c_torque, 10000);
	while(1)
	{set_torque(c_torque, 10000);
		ts when timerafter(time+33333) :> time; //3khz

		/* computed current torque *
		//c_torque<:4;
		//c_torque:>speed;
		speed = get_speed_cal(c_hall_p4);

#ifdef ENABLE_xscope_main
		//xscope_probe_data(0,speed);
		xscope_probe_data(0, acc/1024);
		xscope_probe_data(1, speed);
		//xscope_probe_data(2,joint_torque_actual);
#endif

		acc = ((((speed - prev_speed)*3000)/60)*201)/32;
		//filter_output = filter(filter_buffer, index, filter_length1, acc);
		sp1= (speed*201)/(60*32);
		joint_torque_actual = acc+speed;
	/*	p.speed = speed;
		p.prev_speed = prev_speed;

		// control torque set point
		joint_torque_set = 1300; 		// in coef_prec only
		p.joint_torque_set = joint_torque_set;
		p.previous_error = error_joint_torque_previous;

		// torque controller call c function
		pd_control(p);

		joint_torque_control_out = p.joint_torque_control_out;

		if(joint_torque_control_out >= TORQUE_OUTPUT_MAX) {
			joint_torque_control_out = TORQUE_OUTPUT_MAX;
		}
		else if(joint_torque_control_out < 0){
			joint_torque_control_out = 0;
		}
		set_torque(c_torque, 10000);
		//set_torque(c_torque, joint_torque_control_out);
*
#ifdef ENABLE_xscope_main
		//xscope_probe_data(0,p.joint_torque_actual);
#endif
	//	error_joint_torque_previous = p.error;
		prev_speed = speed;
	}

}*/
