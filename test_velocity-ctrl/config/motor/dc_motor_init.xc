#include <dc_motor_config.h>

void init_hall(hall_par &h_pole)
{
	h_pole.pole_pairs = POLE_PAIRS;
	h_pole.gear_ratio = GEAR_RATIO;
	return;
}

void init_qei(qei_par &qei_params)
{
	qei_params.max_count = QEI_COUNT_MAX;
	qei_params.real_counts = QEI_COUNT_MAX_REAL;
	return;
}

int init_csv(csv_par &csv_params)
{
	csv_params.max_motor_speed = MAX_NOMINAL_SPEED;
	if(POLARITY > 0)
		csv_params.polarity = 1;
	else if(POLARITY < 0)
		csv_params.polarity = -1;
	return 0;
}
