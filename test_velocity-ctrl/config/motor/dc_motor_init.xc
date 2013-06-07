#include <dc_motor_config.h>

void init_hall(hall_par &h_pole)
{
	h_pole.pole_pairs = POLE_PAIRS;
	h_pole.gear_ratio = GEAR_RATIO;
	return;
}

void init_params_struct_all(torq_par &torque, field_par &field, loop_par &loop)
{
	torque.Kp_n = Torque_Kp_n;
	torque.Kp_d = Torque_Kp_d;
	torque.Ki_n = Torque_Ki_n;
	torque.Ki_d = Torque_Ki_d;
	torque.Integral_limit = Torque_Integral_limit;

	torque.Max_torque = Max_torque_out;

	field.Kp_n = Field_Kp_n;
	field.Kp_d = Field_Kp_d;
	field.Ki_n = Field_Ki_n;
	field.Ki_d = Field_Ki_d ;
	field.Integral_limit = Field_Integral_limit;

	loop.delay = loop_timing;

	return;
}

void init_qei(qei_par &qei_params)
{
	qei_params.max_count = QEI_COUNT_MAX;
	qei_params.real_counts = QEI_COUNT_MAX_REAL;
	return;
}
