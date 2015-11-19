/**
 * @file  torque_ctrl_client.h
 * @brief Torque Control Loop Client functions
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <hall_service.h>
#include <qei_service.h>
#include <internal_config.h>
#include "control_loops_common.h"

/**
 * @brief initialize torque control PID params
 *
 * @param torque_ctrl_params struct defines torque control PID params
 */
void init_torque_control_param(ctrl_par &torque_ctrl_params);

/**
 * @brief Initialise Torque Control Loop
 *
 * @Input Channel
 * @param c_torque_ctrl channel to signal initialisation
 */
int init_torque_control(chanend c_torque_ctrl);

/**
 * @brief Checks Torque Control Loop Status
 *
 * @Input Channel
 * @param c_torque_ctrl channel to get state of control loop
 *
 * @Output
 * @return state of the control loop : 1 - active, 0 - inactive
 */
int check_torque_ctrl_state(chanend c_torque_ctrl);

/**
 * @brief Torque Limiter
 *
 * @Input
 * @param torque is the input torque to be limited in range
 * @param max_torque_limit is the max torque that can be reached
 *
 * @Output
 * @return torque in the range [-max_torque_limit to max_torque_limit] (mNm * Current resolution)
 */
int torque_limit(int torque, int max_torque_limit);

/**
 * @brief Get actual torque from Torque Control Loop
 *
 * @Output Channel
 * @param c_torque channel to receive actual torque
 *
 * @Output
 * @return actual torque from torque control in range [0 - mNm * Current Resolution]
 */
int get_torque(chanend c_torque_ctrl);

/**
 * @brief Set new target torque for Torque Control Loop
 *
 * @Input Channel
 * @param c_torque channel to signal new target torque
 *
 * @Input
 * @param torque is the new target torque range [0 - mNm * Current Resolution]
 */
void set_torque(int torque, chanend c_torque_ctrl);

/**
 * @brief Set Torque Control PID Parameters
 *
 * @Input
 * @param torque_ctrl_params struct defines the torque control PID parameters
 */
void set_torque_ctrl_param(ctrl_par & torque_ctrl_params, chanend c_torque_ctrl);

/**
 * @brief Set hall sensor parameters for Torque Control
 *
 * @param hall_params struct defines the pole-pairs and gear ratio
 */
void set_torque_ctrl_hall_param(hall_par & hall_params, chanend c_torque_ctrl);

/**
 * @brief Set QEI sensor parameters for Torque Control
 *
 * @param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_torque_ctrl_qei_param(qei_par & qei_params, chanend c_torque_ctrl);

/**
 * @brief Sets the sensor used for Torque Control
 *
 * @param sensor_used defines the sensor to be used (HALL/QEI) for Torque Control
 */
void set_torque_sensor(int sensor_used, chanend c_torque_ctrl);

/**
 * @brief Enables Torque Control mode operation
 *
 * @param c_torque_ctrl channel to signal enable torque control
 */
void enable_torque_ctrl(chanend c_torque_ctrl);

/**
 * @brief Shutdown Torque Control mode operation
 *
 * @param c_torque_ctrl channel to signal shutdown of torque control
 */
void shutdown_torque_ctrl(chanend c_torque_ctrl);

/**
 * @brief Set new target torque for torque control (advanced function)
 *
 * @Input Channel
 * @param c_torque_ctrl channel to signal new target torque
 *
 * @Input
 * @param cst_param struct defines the motor parameters and torque limits
 * @param target_torque is the new target torque
 * @param torque_offset defines offset in torque
 */
void set_torque_cst(cst_par & cst_params, int target_torque, int torque_offset, chanend c_torque_ctrl);

