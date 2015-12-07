/**
 * @file velocity_ctrl_server.h
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <hall_service.h>
#include <qei_service.h>
#include <internal_config.h>
#include "control_loops_common.h"
#include <motorcontrol_service.h>

interface VelocityControlInterface{
    int check_busy();
    int check_velocity_ctrl_state();
    void set_velocity(int target_velocity);
    int get_velocity();
    void set_velocity_ctrl_param(ControlConfig velocity_ctrl_params);
    void set_velocity_filter(int in_length);
    void set_velocity_ctrl_hall_param(HallConfig hall_config);
    void set_velocity_ctrl_qei_param(QEIConfig qei_params);
    void set_velocity_sensor(int sensor_used);
    void enable_velocity_ctrl();
    void shutdown_velocity_ctrl();
    ControlConfig get_velocity_control_config();
};

#define DEFAULT_FILTER_LENGTH 8
/**
 * @brief Initialise Velocity Control Loop
 *
 * @Input Channel
 * @param c_velocity_ctrl channel to signal initialisation
 */
int init_velocity_control(interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Velocity Limiter
 *
 * @Input
 * @param velocity is the input velocity to be limited in range
 * @param max_speed is the max speed that can be reached
 *
 * @Output
 * @return velocity in the range [-max_speed to max_speed] (rpm)
 */
int max_speed_limit(int velocity, int max_speed);

/**
 * @brief Set new target velocity for velocity control (advanced function)
 *
 * @Input Channel
 * @param c_velocity_ctrl channel to signal new target velocity input
 *
 * @Input
 * @param csv_param struct defines the motor parameters and velocity limits
 * @param target_velocity is the new target velocity
 * @param velocity_offset defines offset in velocity
 * @param torque_offset defines offset in torque
 */
void set_velocity_csv(csv_par & csv_params, int target_velocity,
                      int velocity_offset, int torque_offset, interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Velocity Control Loop
 *
 * @Input
 * @param velocity_ctrl_params struct defines the velocity control parameters
 * @param sensor_filter_par struct defines the filter parameters
 * @param hall_params struct defines the poles for hall sensor and gear-ratio
 * @param qei_params struct defines the resolution for qei sensor and gear-ratio
 * @param sensor_used specify the sensors to used via HALL/QEI defines
 *
 * @Input Channel
 * @param c_hall channel to receive position information from hall
 * @param c_qei channel to receive position information from qei
 * @param c_velocity_ctrl channel to receive/send velocity control information
 *
 * @Output Channel
 * @param c_commutation channel to send motor voltage input value
 *
 */
[[combinable]]
void velocity_control_service(ControlConfig & velocity_ctrl_params,
                        interface HallInterface client ?i_hall,
                        interface QEIInterface client ?i_qei,
                        interface VelocityControlInterface server i_velocity_control,
                        interface MotorcontrolInterface client commutation_interface);

