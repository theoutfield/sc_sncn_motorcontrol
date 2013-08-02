/*
 * hall-qei.h
 *
 *  Created on: Apr 29, 2013
 *      Author: pkanajar
 */

#ifndef HALL_QEI_H_
#define HALL_QEI_H_
#include "dc_motor_config.h"
/**
 *  \channel c_qei qei position data
 *  \channel c_hall1 hall position data
 *  \channel synchronised data from hall and qei
 */
void hall_qei_sync(qei_par &qei_params, hall_par &hall_params, chanend c_qei, chanend c_hall, chanend sync_output);


int get_sync_position ( chanend sync_output );
#endif /* HALL_QEI_H_ */
