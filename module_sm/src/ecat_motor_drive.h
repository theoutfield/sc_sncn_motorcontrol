/*
 * ecat_motor_drive.h
 *
 *  Created on: Sep 13, 2013
 *      Author: pkanajar
 */

#ifndef ECAT_MOTOR_DRIVE_H_
#define ECAT_MOTOR_DRIVE_H_
#include <comm.h>
#include <drive_config.h>
#include <velocity_ctrl.h>
#include <position_ctrl.h>
#include <hall_server.h>
#include <hall_client.h>
#include <qei_client.h>
#include <qei_server.h>
#include <profile.h>

void ecat_motor_drive(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall_p4,\
		chanend c_qei_p4, chanend c_adc, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);

#endif /* ECAT_MOTOR_DRIVE_H_ */
