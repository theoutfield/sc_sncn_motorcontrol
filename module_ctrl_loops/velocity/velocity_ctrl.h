/*
 * velocity_ctrl.h
 *
 *  Created on: Jun 3, 2013
 *      Author: pkanajar
 */

#ifndef VELOCITY_CTRL_H_
#define VELOCITY_CTRL_H_
#include "dc_motor_config.h"
#include <internal_config.h>


/**
 * \brief Initialise Velocity Control Parameters
 *
 *  Input
 * \param velocity_ctrl_par struct defines the velocity control parameters
 */
void init_velocity_control_param(ctrl_par &velocity_ctrl_par);

/**
 * \brief Initialise Velocity Control Loop
 *  Input Channel
 * \channel c_velocity_ctrl channel to signal initialisation
 */
int init_velocity_control(chanend c_velocity_ctrl);

/**
 * \brief Initialise Sensor Filter parameter
 *  Input
 * \param sensor_filter_par struct defines the filter parameters
 */
void init_sensor_filter_param(filter_par &sensor_filter_par);


/**
 * \brief Set new target velocity for Velocity Control
 *
 *  Input Channel
 * \channel c_velocity_ctrl channel to signal new target velocity
 *
 *  Input
 * \param target_velocity is the new target velocity
 */
void set_velocity(int target_velocity, chanend c_velocity_ctrl);

/**
 * \brief Get actual velocity from velocity control
 *
 *  Output Channel
 * \channel c_velocity_ctrl channel to receive actual velocity
 *
 *  Output
 * \return actual velocity from velocity control
 */
int get_velocity(chanend c_velocity_ctrl);


/**
 * \brief Velocity Control Loop
 *
 *  Input
 * \param velocity_ctrl_params struct defines the velocity control parameters
 * \param sensor_filter_par struct defines the filter parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_used specify the sensors to used via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_velocity_ctrl channel to receive/send velocity control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void velocity_control(ctrl_par &velocity_ctrl_params, filter_par &sensor_filter_params, hall_par &hall_params, qei_par &qei_params, \
	 	 	 int sensor_used, chanend c_hall, chanend c_qei, chanend c_velocity_ctrl, chanend c_commutation);

/**
 * \brief Velocity Limiter
 *
 *  Input
 * \param velocity is the input velocity to be limited in range
 * \param max_speed is the max speed that can be reached
 *
 *  Output
 * \return velocity in the range [-max_speed to max_speed]
 */
int max_speed_limit(int velocity, int max_speed);

void set_profile_velocity(int target_velocity, int acceleration, int deceleration, int max_profile_velocity, chanend c_velocity_ctrl);

/* Internal functions for velocity control via ethercat communication */

void set_velocity_csv(csv_par &csv_params, int target_velocity,
		int velocity_offset, int torque_offset, chanend c_velocity_ctrl);

void init_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend c_velocity_ctrl);
void init_velocity_sensor_ecat(int sensor_used, chanend c_velocity_ctrl);
void init_velocity_ctrl_hall(hall_par &hall_params, chanend c_velocity_ctrl);
void init_velocity_ctrl_qei(qei_par &qei_params, chanend c_velocity_ctrl);

void shutdown_velocity_ctrl(chanend c_velocity_ctrl);

void enable_velocity_ctrl(chanend c_velocity_ctrl);
#endif /* VELOCITY_CTRL_H_ */
