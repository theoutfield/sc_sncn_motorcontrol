/*
 * velocity_ctrl.h
 *
 *  Created on: Jun 3, 2013
 *      Author: pkanajar
 */

#ifndef VELOCITY_CTRL_H_
#define VELOCITY_CTRL_H_
#include "dc_motor_config.h"
#include <internal_config.h>

void init_velocity_control_param(ctrl_par &velocity_ctrl_par);

int init_velocity_control(chanend c_velocity_ctrl);

void init_sensor_filter_param(filt_par &sensor_filter_par);


//internal
void set_velocity(int target_velocity, chanend c_velocity_ctrl);

int get_velocity(chanend c_velocity_ctrl);

int max_speed_limit(int velocity, int max_speed);

//csv mode function
void set_velocity_csv(csv_par &csv_params, int target_velocity,
		int velocity_offset, int torque_offset, chanend c_velocity_ctrl);

//controller
void velocity_control(ctrl_par &velocity_ctrl_params, filt_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params,\
		int sensor_used, chanend c_hall, chanend c_qei, chanend c_velocity_ctrl, chanend c_commutation);


//ethercat
void init_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend c_velocity_ctrl);
void init_velocity_sensor_ecat(int sensor_used, chanend c_velocity_ctrl);




void shutdown_velocity_ctrl(chanend c_velocity_ctrl);

void enable_velocity_ctrl(chanend c_velocity_ctrl);
#endif /* VELOCITY_CTRL_H_ */
