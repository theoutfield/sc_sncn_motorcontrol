#include <dc_motor_config.h>
#include "refclk.h"

extern int __qei_max_counts(int real_counts);

void init_hall_param(hall_par &hall_params)
{
	hall_params.pole_pairs = POLE_PAIRS;
	hall_params.gear_ratio = GEAR_RATIO;
	return;
}

void init_qei_param(qei_par &qei_params)
{
	qei_params.real_counts = QEI_COUNT_MAX_REAL;
	qei_params.gear_ratio = GEAR_RATIO;
	qei_params.index = QEI_SENSOR_TYPE;
	qei_params.max_count = __qei_max_counts(qei_params.real_counts);
	return;
}

void init_csv_param(csv_par &csv_params)
{
	csv_params.max_motor_speed = MAX_NOMINAL_SPEED;
	if(POLARITY >= 0)
		csv_params.polarity = 1;
	else if(POLARITY < 0)
		csv_params.polarity = -1;
	return;
}

void init_csp_param(csp_par &csp_params)
{
	csp_params.base.max_motor_speed = MAX_NOMINAL_SPEED;
	if(POLARITY >= 0)
		csp_params.base.polarity = 1;
	else if(POLARITY < 0)
		csp_params.base.polarity = -1;
	csp_params.max_following_error = MAX_FOLLOWING_ERROR;
	csp_params.max_position_limit = MAX_POSITION_LIMIT;
	csp_params.min_position_limit = MIN_POSITION_LIMIT;
	return;
}

void init_velocity_control_param(ctrl_par &velocity_ctrl_params)
{
	velocity_ctrl_params.Kp_n = VELOCITY_KP;
	velocity_ctrl_params.Kp_d = 16384;
	velocity_ctrl_params.Ki_n = VELOCITY_KI;
	velocity_ctrl_params.Ki_d = 16384;
	velocity_ctrl_params.Kd_n = VELOCITY_KD;
	velocity_ctrl_params.Kd_d = 16384;
	velocity_ctrl_params.Loop_time = 1 * MSEC_STD;  //units - core timer value //CORE 2/1/0 default

	velocity_ctrl_params.Control_limit = 13739; //default

	if(velocity_ctrl_params.Ki_n != 0)    							//auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = velocity_ctrl_params.Control_limit * (velocity_ctrl_params.Ki_d/velocity_ctrl_params.Ki_n );
	else
		velocity_ctrl_params.Integral_limit = 0;

	return;
}

void init_position_control_param(ctrl_par &position_ctrl_params)
{

	position_ctrl_params.Kp_n = POSITION_KP;
	position_ctrl_params.Kp_d = 16384;
	position_ctrl_params.Ki_n = POSITION_KI;
	position_ctrl_params.Ki_d = 16384;
	position_ctrl_params.Kd_n = POSITION_KD;
	position_ctrl_params.Kd_d = 16384;
	position_ctrl_params.Loop_time = 1 * MSEC_STD;  // units - for CORE 2/1/0 only default

	position_ctrl_params.Control_limit = 13739; 							 // default do not change

	if(position_ctrl_params.Ki_n != 0)										 // auto calculated using control_limit
	{
		position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n) ;
	}
	else
	{
		position_ctrl_params.Integral_limit = 0;
	}

	return;
}
