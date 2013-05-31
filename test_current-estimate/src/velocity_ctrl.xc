#include "dc_motor_config.h"
#include "refclk.h"
//default runs on CORE 2/CORE 1/CORE 0


#define VELOCITY_Kp_NUMERATOR 	25
#define VELOCITY_Kp_DENOMINATOR 10
#define VELOCITY_Ki_NUMERATOR   15
#define VELOCITY_Ki_DENOMINATOR 100
#define VELOCITY_Kd_NUMERATOR   0
#define VELOCITY_Kd_DENOMINATOR 1
#define VELOCITY_CONTROL_LOOP_TIME 1			//in ms

#define FILTER_SIZE 8                           //optional

#define VELOCITY_CONTROL_LIMIT 13739			// Default
#define FILTER_SIZE_MAX 16



void init_velocity_control(ctrl_par &velocity_ctrl_par)
{
	velocity_ctrl_par.Kp_n = VELOCITY_Kp_NUMERATOR;
	velocity_ctrl_par.Kp_d = VELOCITY_Kp_DENOMINATOR;
	velocity_ctrl_par.Ki_n = VELOCITY_Ki_NUMERATOR;
	velocity_ctrl_par.Ki_d = VELOCITY_Ki_DENOMINATOR;
	velocity_ctrl_par.Kd_n = VELOCITY_Kd_NUMERATOR;
	velocity_ctrl_par.Kd_d = VELOCITY_Kd_DENOMINATOR;
	velocity_ctrl_par.Loop_time = VELOCITY_CONTROL_LOOP_TIME * MSEC_STD;   //CORE 2/1/0

	velocity_ctrl_par.Control_limit = VELOCITY_CONTROL_LIMIT;

	if(velocity_ctrl_par.Ki_n != 0)    							//auto calculated using control_limit
		velocity_ctrl_par.Integral_limit = (velocity_ctrl_par.Control_limit * velocity_ctrl_par.Ki_d)/velocity_ctrl_par.Ki_n ;
	else
		velocity_ctrl_par.Integral_limit = 0;

	return;
}

void init_sensor_filter(filt_par &sensor_filter_par) //optional for user to change
{
	sensor_filter_par.filter_length = FILTER_SIZE;
	return;
}

void velocity_control(ctrl_par &velocity_ctrl_par, filt_par &sensor_filter_par, chanend c_hall, chanend c_qei, chanend c_commutation)
{
	/* Controller defines */
	int actual_velocity = 0;
	int target_velocity = 700;
	int error_velocity = 0;
	int error_velocity_D = 0;
	int error_velocity_I = 0;
	int previous_error = 0;
	int velocity_control_out = 0;

	timer ts;
	unsigned int time;

	/* Sensor filter defines */
	int filter_length = sensor_filter_par.filter_length;
	int filter_buffer[FILTER_SIZE_MAX];						//default size used at compile time (cant be changed further)
	int index = 0;
	int filter_output;
	int old_filter_output = 0;

	ts :> time;
	while(1)
	{
		ts when timerafter(time + velocity_ctrl_par.Loop_time) :> time;

		/* acq actual velocity hall/qei with filter*/

		error_velocity   = (target_velocity - actual_velocity)*1000;
		error_velocity_I = error_velocity_I + error_velocity;
		error_velocity_D = error_velocity - previous_error;

		if(error_velocity_I > (velocity_ctrl_par.Integral_limit)*1000)
			error_velocity_I = (velocity_ctrl_par.Integral_limit)*1000;

		velocity_control_out = (velocity_ctrl_par.Kp_n*error_velocity)/(1000*velocity_ctrl_par.Kp_d) + (velocity_ctrl_par.Ki_n*error_velocity_I)/(1000 * velocity_ctrl_par.Ki_d) \
								+ (velocity_ctrl_par.Kd_n*error_velocity_D)/(1000 * velocity_ctrl_par.Kd_d);

		if(velocity_control_out > velocity_ctrl_par.Control_limit)
			velocity_control_out = velocity_ctrl_par.Control_limit;

		previous_error = error_velocity;

		/* acq target velocity etherCAT */
	}


}
