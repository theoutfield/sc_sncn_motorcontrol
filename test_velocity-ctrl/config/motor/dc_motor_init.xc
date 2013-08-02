#include <dc_motor_config.h>

extern int __qei_max_counts(int real_counts);

void init_hall_param(hall_par &h_pole)
{
	h_pole.pole_pairs = POLE_PAIRS;
	h_pole.gear_ratio = GEAR_RATIO;
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


