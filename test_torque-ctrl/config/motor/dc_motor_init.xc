#include <dc_motor_config.h>

extern int __qei_max_counts(int real_counts);

void init_commutation_param(commutation_par &commutation_params)
{
	commutation_params.angle_offset_clkwise = COMMUTATION_ANGLE_OFFSET_CLOCKWISE;
	commutation_params.angle_offset_cclkwise = COMMUTATION_ANGLE_OFFSET_COUNTERCLOCKWISE;
}

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

