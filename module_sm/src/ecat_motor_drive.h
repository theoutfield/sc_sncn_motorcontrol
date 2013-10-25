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

/**
 * \brief This server implementation enables motor drive functions via Ethercat communication
 *
 *  Input Channel
 * \channel pdo_in channel to receive information from ethercat
 * \channel coe_out channel to receive motor config information from ethercat
 * \channel c_signal channel to receive init ack from commutation loop
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_adc channel to receive torque information from adc
 *
 *  Output Channel
 * \channel pdo_out channel to send out information via ethercat
 * \channel c_position_ctrl channel to receive actual position
 * \channel c_torque_ctrl channel to receive/send torque control information
 * \channel c_velocity_ctrl channel to receive/send velocity control information
 * \channel c_position_ctrl channel to receive/send position control information
 *
 */
void ecat_motor_drive(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall,\
		chanend c_qei, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);

#endif /* ECAT_MOTOR_DRIVE_H_ */
