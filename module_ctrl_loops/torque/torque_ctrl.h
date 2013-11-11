
/**
 * \file torque_ctrl.h
 *
 *	Torque Control Loop
 *
 * Copyright 2013, Synapticon GmbH. All rights reserved.
 * Authors: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the
 * copyright notice above.
 *
 **/

#include <comm_loop.h>
#include <dc_motor_config.h>
#include <internal_config.h>

/**
 * \brief Initialise Torque Control Loop
 *  Input Channel
 * \channel c_torque_ctrl channel to signal initialisation
 */
int init_torque_control(chanend c_torque_ctrl);

/**
 * \brief Torque Limiter
 *
 *  Input
 * \param torque is the input torque to be limited in range
 * \param max_torque_limit is the max torque that can be reached
 *
 *  Output
 * \return torque in the range [-max_torque_limit to max_torque_limit]
 */
int torque_limit(int torque, int max_torque_limit);

/**
 * \brief Get actual torque from Torque Control Loop
 *
 *  Output Channel
 * \channel c_torque channel to receive actual torque
 *
 *  Output
 * \return actual torque from torque control in range [0 - mNm * Current Resolution]
 */
int get_torque(cst_par &cst_params, chanend c_torque_ctrl);

/**
 * \brief Set new target torque for Torque Control Loop
 *
 *  Input Channel
 * \channel c_torque channel to signal new target torque
 *
 *  Input
 * \param torque is the new target torque range [0 - mNm * Current Resolution]
 */
void set_torque(int torque,  cst_par &cst_params, chanend c_torque_ctrl);

/**
 * \brief Set Torque Control PID Parameters
 *
 *  Input
 * \param torque_ctrl_params struct defines the torque control PID parameters
 */
void set_torque_ctrl_param(ctrl_par &torque_ctrl_params, chanend c_torque_ctrl);

/**
 * \brief initialize hall sensor parameters for Torque Control
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void set_torque_ctrl_hall_param(hall_par &hall_params, chanend c_torque_ctrl);

/**
 * \brief initialize QEI sensor for Torque Control
 *
 * \param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_torque_ctrl_qei_param(qei_par &qei_params, chanend c_torque_ctrl);

/**
 * \brief Sets the sensor used for Torque Control
 *
 * \param sensor_used defines the sensor to be used (HALL/QEI) for Torque Control
 */
void set_torque_sensor(int sensor_used, chanend c_torque_ctrl);

/**
 * \brief Enables Torque Control mode operation
 *
 * \channel c_torque_ctrl channel to signal enable torque control
 */
void enable_torque_ctrl(chanend c_torque_ctrl);

/**
 * \brief Shutdown Torque Control mode operation
 *
 * \channel c_torque_ctrl channel to signal shutdown of torque control
 */
void shutdown_torque_ctrl(chanend c_torque_ctrl);


/**
 * \brief Set new target torque for torque control
 *
 *  Input Channel
 * \channel c_torque_ctrl channel to signal new target torque
 *
 *  Input
 * \param cst_param struct defines the motor parameters and torque limits
 * \param target_torque is the new target torque
 * \param torque_offset defines offset in torque
 */
void set_torque_cst(cst_par &cst_params, int target_torque, int torque_offset, chanend c_torque_ctrl);



/**
 * \brief Torque Control Loop
 *
 *  Input
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 *
 *  Input Channel
 * \channel c_adc channel to receive to torque information from current sensor
 * \channel c_hall channel to receive position information from hall
 * \channel sync_output channel to receive synced position information from qei synced with hall
 * \channel c_torque channel to receive/send torque control information
 *
 *  Output Channel
 * \channel signal_adc channel for signaling to start adc after initialization
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void torque_control(ctrl_par &torque_ctrl_params, hall_par &hall_params, qei_par &qei_params, \
		chanend c_adc, chanend c_commutation, chanend c_hall, chanend c_qei, chanend c_torque_ctrl);



