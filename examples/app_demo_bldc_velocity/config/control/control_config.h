/*
 * control_config.h
 *
 *  Created on: Nov 30, 2015
 *      Author: atena
 */


#pragma once

#include <control_loops_common.h>
/**
 * @brief initialize position control PID params
 *
 * @param position_ctrl_params struct defines position control PID params
 */
void init_velocity_control_config(ControlConfig &velocity_ctrl_params);
