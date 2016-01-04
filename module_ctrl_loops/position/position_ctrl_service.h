/**
 * @file  position_ctrl_server.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

//#include <profile_control.h>
#include <qei_service.h>
#include <motorcontrol_service.h>
#include <control_loops_common.h>

/**
 * @brief Lorem ipsum...
 */
interface PositionControlInterface{
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int check_busy();
    /**
     * @brief Lorem ipsum...
     *
     * @param target_position Lorem ipsum...
     */
    void set_position(int target_position);
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_position();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_target_position();
    /**
     * @brief Lorem ipsum...
     *
     * @param position_ctrl_params Lorem ipsum...
     */
    void set_position_ctrl_param(ControlConfig position_ctrl_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param hall_config Lorem ipsum...
     */
    void set_position_ctrl_hall_param(HallConfig hall_config);
    /**
     * @brief Lorem ipsum...
     *
     * @param qei_params Lorem ipsum...
     */
    void set_position_ctrl_qei_param(QEIConfig qei_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param sensor_used Lorem ipsum...
     */
    void set_position_sensor(int sensor_used);
    /**
     * @brief Lorem ipsum...
     */
    void enable_position_ctrl();
    /**
     * @brief Lorem ipsum...
     */
    void shutdown_position_ctrl();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int check_position_ctrl_state();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    ControlConfig get_control_config();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    HallConfig get_hall_config();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    QEIConfig get_qei_config();
};

/**
 * @brief Initialise Position Control Loop
 *
 * @param i_position_control channel to signal initialisation
 *
 * @return Lorem ipsum...
 */
int init_position_control(interface PositionControlInterface client i_position_control);

/**
 * @brief Position Limiter
 *
 * @Input
 * @param position is the input position to be limited in range
 * @param max_position_limit is the max position that can be reached
 * @param min_position_limit is the min position that can be reached
 *
 * @Output
 * @return position in the range [min_position_limit - max_position_limit]
 */
int position_limit(int position, int max_position_limit, int min_position_limit);

/**
 * @brief Set new target position for position control (advanced function)
 *
 * @Input
 * @param csp_params struct defines the motor parameters and position limits
 * @param target_position is the new target position
 * @param position_offset defines offset in position
 * @param velocity_offset defines offset in velocity
 * @param torque_offset defines offset in torque
 * @param i_position_control Lorem ipsum...
 */
void set_position_csp(ProfilerConfig &csp_params, int target_position, int position_offset, int velocity_offset,
                      int torque_offset, interface PositionControlInterface client i_position_control);


/**
 * @brief Position Control Loop
 *  Implements PID controller for position using Hall or QEI sensors.
 *  Note: The Server must be placed on CORES 0/1/2 only.
 *
 * @param position_ctrl_params struct defines the position control parameters
 * @param i_hall Lorem ipsum...
 * @param i_qei Lorem ipsum...
 * @param i_commutation Lorem ipsum...
 * @param i_position_control[3] Lorem ipsum...
 *
 */
void position_control_service(ControlConfig & position_ctrl_params,
                    interface HallInterface client ?i_hall,
                    interface QEIInterface client ?i_qei,
                    interface MotorcontrolInterface client i_commutation,
                    interface PositionControlInterface server i_position_control[3]);

