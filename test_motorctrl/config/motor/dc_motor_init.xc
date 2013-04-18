#include <dc_motor_config.h>

void init_hall(hall_par &h_pole)
{
	h_pole.pole_pairs = POLE_PAIRS;
	return;
}

void init_params_struct_all(torq_par &tor, field_par &field, loop_par &loop)
{
	tor.Kp_n = Torque_Kp_n;
	tor.Kp_d = Torque_Kp_d;
	tor.Ki_n = Torque_Ki_n;
	tor.Ki_d = Torque_Ki_d;
	tor.Integral_limit = Torque_Integral_limit;

	tor.Max_torque = Max_torque_out;

	field.Kp_n = Field_Kp_n;
	field.Kp_d = Field_Kp_d;
	field.Ki_n = Field_Ki_n;
	field.Ki_d = Field_Ki_d ;
	field.Integral_limit = Field_Integral_limit;

	loop.delay = loop_timing;

	return;
}

void init_qei(qei_par &q_max)
{
	q_max.max_count = QEI_COUNT_MAX;
	return;
}
