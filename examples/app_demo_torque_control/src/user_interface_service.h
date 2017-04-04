/*
 * tuning.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Synapticon GmbH
 */


#pragma once

#include <motor_control_interfaces.h>

/*
 * The following service shows how to directly work with module_torque_control.
 * It is able to:
 *  - automatically find motor offset
 *  - read/set motor offsett
 *  - enable/disable torque controller
 *  - send the reference value of the torque to motor_control_service
 *  - lock/unlock the brakes
 *
 * @param i_motorcontrol -> interface of type MotorControlInterface to communicate with torque controller
 */
void demo_torque_control(interface MotorControlInterface client i_motorcontrol);
