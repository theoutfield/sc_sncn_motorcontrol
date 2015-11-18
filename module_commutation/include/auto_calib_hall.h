/*
 * auto_calib_hall.h
 *
 *  Created on: Oct 4, 2013
 *      Author: pkanajar
 */

#pragma once

#include <hall_client.h>
#include <qei_client.h>
#include <commutation_client.h>
#include <refclk.h>
#include <velocity_ctrl_client.h>
#include <internal_config.h>
#include <statemachine.h>
#include <drive_modes.h>

int get_average_velocity(int sensor_select, chanend c_hall,
                         HallConfig & hall_config,
                         qei_velocity_par & qei_velocity_params,
                         int core_id, timer t, int & avg_times,
                         chanend c_qei,
                         qei_par & qei_params);

void ramp_up(int & i, int comm_voltage, timer t, int core_id, chanend c_commutation);

void ramp_down(int & i, int comm_voltage, timer t, int core_id, chanend c_commutation);

{int, int} update_comm_sine_max_state(int &sensor_select, timer t, int core_id,
                                      HallConfig & hall_config, qei_velocity_par & qei_velocity_params,
                                      int & avg_times, int max, chanend c_hall, chanend c_qei,
                                      qei_par &qei_params);

void commutation_sine_automate(int & sensor_select, chanend c_signal, chanend c_commutation,
                               commutation_par &commutation_params, HallConfig &hall_config,
                               qei_par &qei_params,
                               chanend c_hall, chanend c_qei);
