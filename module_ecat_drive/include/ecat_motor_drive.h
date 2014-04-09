
/**
 * \file ecat_motor_drive.h
 * \brief Ethercat Motor Drive Server
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */
/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#ifndef ECAT_MOTOR_DRIVE_H_
#define ECAT_MOTOR_DRIVE_H_
#include <comm.h>
#include <drive_config.h>
#include <velocity_ctrl_client.h>
#include <position_ctrl_client.h>
#include <torque_ctrl_client.h>
#include <hall_client.h>
#include <qei_client.h>
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
 *
 *  Output Channel
 * \channel pdo_out channel to send out information via ethercat
 * \channel c_torque_ctrl channel to receive/send torque control information
 * \channel c_velocity_ctrl channel to receive/send velocity control information
 * \channel c_position_ctrl channel to receive/send position control information
 * \channel c_gpio channel to config/read/drive GPIO digital ports
 *
 */
void ecat_motor_drive(chanend pdo_out, chanend pdo_in, chanend coe_out, chanend c_signal, chanend c_hall,\
		chanend c_qei, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl, chanend c_gpio);

int detect_sensor_placement(chanend c_hall, chanend c_qei, chanend c_commutation);
#endif /* ECAT_MOTOR_DRIVE_H_ */
