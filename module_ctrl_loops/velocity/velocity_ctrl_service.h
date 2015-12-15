/**
 * @file velocity_ctrl_server.h
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <hall_service.h>
#include <qei_service.h>
#include <motorcontrol_service.h>
#include <control_loops_common.h>

/**
 * @brief Lorem ipsum...
 */
interface VelocityControlInterface{
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int check_busy();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int check_velocity_ctrl_state();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_velocity();
    /**
     * @brief Lorem ipsum...
     *
     * @param target_velocity Lorem ipsum...
     */
    void set_velocity(int target_velocity);
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_set_velocity();
    /**
     * @brief Lorem ipsum...
     *
     * @param velocity_ctrl_params Lorem ipsum...
     */
    void set_velocity_ctrl_param(ControlConfig velocity_ctrl_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param in_length Lorem ipsum...
     */
    void set_velocity_filter(int in_length);
    /**
     * @brief Lorem ipsum...
     *
     * @param hall_config Lorem ipsum...
     */
    void set_velocity_ctrl_hall_param(HallConfig hall_config);
    /**
     * @brief Lorem ipsum...
     *
     * @param qei_params Lorem ipsum...
     */
    void set_velocity_ctrl_qei_param(QEIConfig qei_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param sensor_used Lorem ipsum...
     */
    void set_velocity_sensor(int sensor_used);
    /**
     * @brief Lorem ipsum...
     */
    void enable_velocity_ctrl();
    /**
     * @brief Lorem ipsum...
     */
    void shutdown_velocity_ctrl();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    ControlConfig get_velocity_control_config();
};

#define DEFAULT_FILTER_LENGTH 8
/**
 * @brief Initialise Velocity Control Loop
 *
 * @Input Channel
 * @param i_velocity_control channel to signal initialisation
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
 * @Input
 * @param csv_params struct defines the motor parameters and velocity limits
 * @param target_velocity is the new target velocity
 * @param velocity_offset defines offset in velocity
 * @param torque_offset defines offset in torque
 * @param i_velocity_control Lorem ipsum...
 */
void set_velocity_csv(CyclicSyncVelocityConfig & csv_params, int target_velocity,
                      int velocity_offset, int torque_offset, interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Velocity Control Loop
 *
 * @param velocity_ctrl_params struct defines the velocity control parameters
 * @param i_hall Lorem ipsum...
 * @param i_qei Lorem ipsum...
 * @param commutation_interface Lorem ipsum...
 * @param i_velocity_control[3] Lorem ipsum...
 *
 */
[[combinable]]
void velocity_control_service(ControlConfig & velocity_ctrl_params,
                        interface HallInterface client ?i_hall,
                        interface QEIInterface client ?i_qei,
                        interface MotorcontrolInterface client commutation_interface,
                        interface VelocityControlInterface server i_velocity_control[3]);

