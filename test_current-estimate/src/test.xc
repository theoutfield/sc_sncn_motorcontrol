#include "test.h"
#include <xs1.h>
#include <platform.h>
#include "refclk.h"

void set_torque_test(chanend c_torque) {
	int torque;
	in_data d;
	while (1) {
		input_cmd(d);
		//printintln(d.set_torque);

		c_torque <: 2;
		c_torque <: d.set_torque;

	}
}
