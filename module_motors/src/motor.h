
/**
 * \file motor.h
 * \brief Initialized motor parameters. Please a desired motor with #include<your_motor.h>
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

#ifndef MOTOR_H_
#define MOTOR_H_
#include <print.h>
#include <internal_config.h>
#include <motor_maxon_313320.h>
#include <motor_generic_bldc.h>

#pragma once

#define IFM_RESOLUTION              DC100_RESOLUTION
#define VELOCITY_FILTER_SIZE        24      //default (could be changed upto 128)

/**
 * \brief Initialize Velocity sensor filter
 *
 * \param sensor_filter_par struct defines the velocity filter params
 */
void init_sensor_filter_param(filter_par &sensor_filter_par) ;

/**
 * \brief Initialize QEI sensor
 *
 * \param qei_params struct defines the resolution for quadrature encoder (QEI),
 *          gear-ratio, poles, encoder type
 */
void init_qei_param(qei_par &qei_params);

/**
 * \brief initialize hall sensor
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void init_hall_param(hall_par &hall_params);

/**
 * \brief Initialize cyclic synchronous velocity params
 *
 * \param csv_params struct defines cyclic synchronous velocity params
 */
void init_csv_param(csv_par &csv_params);

/**
 * \brief Initialize cyclic synchronous position params
 *
 * \param csp_params struct defines cyclic synchronous position params
 */
void init_csp_param(csp_par &csp_params);

/**
 * \brief Initialize cyclic synchronous torque params
 *
 * \param cst_params struct defines cyclic synchronous torque params
 */
void init_cst_param(cst_par &cst_params);

/**
 * \brief Initialize profile position params
 *
 * \param pp_params struct defines profileposition params
 */
void init_pp_params(pp_par &pp_params);

/**
 * \brief Initialize profile velocity params
 *
 * \param pv_params struct defines profile velocity params
 */
void init_pv_params(pv_par &pv_params);

/**
 * \brief Initialize profile torque params
 *
 * \param pt_params struct defines profile torque params
 */
void init_pt_params(pt_par &pt_params);

/**
 * \brief Initialize torque control PID params
 *
 * \param torque_ctrl_params struct defines torque control PID params
 */
void init_torque_control_param(ctrl_par &torque_ctrl_params);

/**
 * \brief Initialize velocity control PID params
 *
 * \param velocity_ctrl_params struct defines velocity control PID params
 */
void init_velocity_control_param(ctrl_par &velocity_ctrl_params);

/**
 * \brief Initialize position control PID params
 *
 * \param position_ctrl_params struct defines position control PID params
 */
void init_position_control_param(ctrl_par &position_ctrl_params);

#endif
