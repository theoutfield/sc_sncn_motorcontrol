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
	qei_params.poles = POLE_PAIRS;
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
	csp_params.max_following_error = 0;
	csp_params.max_position_limit = MAX_POSITION_LIMIT;
	csp_params.min_position_limit = MIN_POSITION_LIMIT;
	return;
}

void init_pt_params(pt_par &pt_params)
{
	pt_params.profile_slope = PROFILE_TORQUE_SLOPE;
	pt_params.polarity = POLARITY;
}

void init_pp_params(pp_par &pp_params)
{
	pp_params.base.max_profile_velocity = MAX_PROFILE_VELOCITY;
	pp_params.profile_velocity	= PROFILE_VELOCITY;
	pp_params.base.profile_acceleration = PROFILE_ACCELERATION;
	pp_params.base.profile_deceleration = PROFILE_DECELERATION;
	pp_params.base.quick_stop_deceleration = QUICK_STOP_DECELERATION;
	pp_params.max_acceleration = MAX_ACCELERATION;
	pp_params.base.polarity = POLARITY;
	pp_params.software_position_limit_max = MAX_POSITION_LIMIT;
	pp_params.software_position_limit_min = MIN_POSITION_LIMIT;
	return;
}

void init_pv_params(pv_par &pv_params)
{
	pv_params.max_profile_velocity = MAX_PROFILE_VELOCITY;
	pv_params.profile_acceleration = PROFILE_ACCELERATION;
	pv_params.profile_deceleration = PROFILE_DECELERATION;
	pv_params.quick_stop_deceleration = QUICK_STOP_DECELERATION;
	pv_params.polarity = POLARITY;
	return;
}

void init_velocity_control_param(ctrl_par &velocity_ctrl_params)
{
	velocity_ctrl_params.Kp_n = VELOCITY_Kp_NUMERATOR;
	velocity_ctrl_params.Kp_d = VELOCITY_Kp_DENOMINATOR;
	velocity_ctrl_params.Ki_n = VELOCITY_Ki_NUMERATOR;
	velocity_ctrl_params.Ki_d = VELOCITY_Ki_DENOMINATOR;
	velocity_ctrl_params.Kd_n = VELOCITY_Kd_NUMERATOR;
	velocity_ctrl_params.Kd_d = VELOCITY_Kd_DENOMINATOR;
	velocity_ctrl_params.Loop_time = 1 * MSEC_STD;  //units - core timer value //CORE 2/1/0 default

	velocity_ctrl_params.Control_limit = 13739; //default

	if(velocity_ctrl_params.Ki_n != 0)    							//auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = velocity_ctrl_params.Control_limit * (velocity_ctrl_params.Ki_d/velocity_ctrl_params.Ki_n) ;
	else
		velocity_ctrl_params.Integral_limit = 0;

	return;
}

void init_position_control_param(ctrl_par &position_ctrl_params)
{

	position_ctrl_params.Kp_n = POSITION_Kp_NUMERATOR;
	position_ctrl_params.Kp_d = POSITION_Kp_DENOMINATOR;
	position_ctrl_params.Ki_n = POSITION_Ki_NUMERATOR;
	position_ctrl_params.Ki_d = POSITION_Ki_DENOMINATOR;
	position_ctrl_params.Kd_n = POSITION_Kd_NUMERATOR;
	position_ctrl_params.Kd_d = POSITION_Kd_DENOMINATOR;
	position_ctrl_params.Loop_time = 1 * MSEC_STD;  // units - for CORE 2/1/0 only default

	position_ctrl_params.Control_limit = 13739; 							 // default do not change

	if(position_ctrl_params.Ki_n != 0)										 // auto calculated using control_limit
	{
		position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n);
	}
	else
	{
		position_ctrl_params.Integral_limit = 0;
	}

	return;
}

void init_torque_control_param(ctrl_par &torque_ctrl_params)
{

	torque_ctrl_params.Kp_n = TORQUE_Kp_NUMERATOR;
	torque_ctrl_params.Kp_d = TORQUE_Kp_DENOMINATOR;
	torque_ctrl_params.Ki_n = TORQUE_Ki_NUMERATOR;
	torque_ctrl_params.Ki_d = TORQUE_Ki_DENOMINATOR;
	torque_ctrl_params.Kd_n = TORQUE_Kd_NUMERATOR;
	torque_ctrl_params.Kd_d = TORQUE_Kd_DENOMINATOR;
	torque_ctrl_params.Loop_time = 1 * MSEC_STD;  // units - for CORE 2/1/0 only default

	torque_ctrl_params.Control_limit = 13739; 							 // default do not change

	if(torque_ctrl_params.Ki_n != 0)										 // auto calculated using control_limit
	{
		torque_ctrl_params.Integral_limit = torque_ctrl_params.Control_limit * (torque_ctrl_params.Ki_d/torque_ctrl_params.Ki_n);
	}
	else
	{
		torque_ctrl_params.Integral_limit = 0;
	}

	return;
}

void init_cst_param(cst_par &cst_params)
{
	cst_params.nominal_current = MAX_NOMINAL_CURRENT;
	cst_params.nominal_motor_speed = MAX_NOMINAL_SPEED;
	cst_params.polarity = POLARITY;
	cst_params.max_torque = MAX_NOMINAL_CURRENT * IFM_RESOLUTION;
}

void init_sensor_filter_param(filter_par &sensor_filter_par) //optional for user to change
{
	sensor_filter_par.filter_length = VELOCITY_FILTER_SIZE;
	return;
}
