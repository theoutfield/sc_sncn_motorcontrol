#include <dc_motor_config.h>
#include <internal_config.h>
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop.h"

int init_position_control(chanend c_position_ctrl);

void init_position_control_param(ctrl_par &position_ctrl_params);

void set_position(int target_position, chanend c_position_ctrl);

int position_limit(int position, int max_position_limit, int min_position_limit);

int get_position(chanend c_position_ctrl);

void set_position_csp(csp_par &csp_params, int target_position, int position_offset, int velocity_offset,\
		              int torque_offset, chanend c_position_ctrl);

void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_position_ctrl, chanend c_commutation);

