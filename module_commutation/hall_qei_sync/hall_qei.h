/*
 * hall-qei.h
 *
 *  Created on: Apr 29, 2013
 *      Author: pkanajar
 */

#ifndef HALL_QEI_H_
#define HALL_QEI_H_
#include "dc_motor_config.h"
#include "comm_loop.h"
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "xscope.h"

/**
 *  \channel c_qei qei position data
 *  \channel c_hall1 hall position data
 *  \channel synchronised data from hall and qei
 */
void hall_qei_sync(qei_par &qei_params, hall_par &hall_params, commutation_par &commutation_params, chanend c_qei, chanend c_hall, chanend sync_output, chanend c_calib);

int get_sync_position ( chanend sync_output );

void calib_qei(int select_direction, commutation_par &commutation_params, timer t, chanend c_calib);

void set_qei_offset(commutation_par &commutation_params, chanend c_calib);

#endif /* HALL_QEI_H_ */
