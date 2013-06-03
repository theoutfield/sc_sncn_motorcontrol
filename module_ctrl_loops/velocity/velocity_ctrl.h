/*
 * velocity_ctrl.h
 *
 *  Created on: Jun 3, 2013
 *      Author: pkanajar
 */

#ifndef VELOCITY_CTRL_H_
#define VELOCITY_CTRL_H_
#include "dc_motor_config.h"

void init_velocity_control(ctrl_par &velocity_ctrl_par);

void init_sensor_filter(filt_par &sensor_filter_par);

void velocity_control(ctrl_par &velocity_ctrl_params, filt_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, chanend c_hall, chanend c_qei, chanend c_commutation);

#endif /* VELOCITY_CTRL_H_ */
