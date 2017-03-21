/**
 * @file  motion_control_service.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <control_loops_common.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>

#define POSITION_LIMIT_THRESHOLD        20000   // threshold in ticks to re-enable the position controler if the limit reached
#define POSITION_CONTROL_LOOP_PERIOD    1000    /**< Period for the control loop [microseconds]. */

/**
 * @brief Interface type to communicate with the Position Control Service.
 */
interface PositionVelocityCtrlInterface{

    void disable();

    void enable_position_ctrl(int pos_control_mode_);
//    void set_position(int in_target_position);
//    void set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//    void set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
//    void set_position_limits(int position_min_limit, int position_max_limit);

    void enable_velocity_ctrl(void);
//    void set_velocity(int in_target_velocity);
//    void set_offset_torque(int offset_torque_);
//    void set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//    void set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
//    void set_velocity_limits(int velocity_min_limit, int velocity_max_limit);

    /**
     * @brief (internal) Settings to suppress the overshoot
     *
     */
    void set_j(int j);

    void enable_torque_ctrl();
    void set_torque(int in_target_torque);
//    void set_torque_limits(int torque_min_limit, int torque_max_limit);


    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    PosVelocityControlConfig get_position_velocity_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_position_velocity_control_config(PosVelocityControlConfig in_config);

    /**
     * @brief Setter for new configuration in the Motorcontrol Service.
     *
     * @param in_config New Service configuration.
     */
    void set_motorcontrol_config(MotorcontrolConfig in_config);

    /**
     * @brief Getter for current configuration used by the Motorcontrol Service.
     *
     * @return Current Service configuration.
     */
    MotorcontrolConfig get_motorcontrol_config();

    /**
     * @brief Sets brake status to ON (no movement) or OFF (possible to move)
     */
    void set_brake_status(int brake_status);


    void update_brake_configuration();

    /**
     * @brief Enables the offset detection process
     */
    MotorcontrolConfig set_offset_detection_enabled();

    /**
     * @brief Send a reset fault command to the motorcontrol
     */
    void reset_motorcontrol_faults();

    int get_position();

    int get_velocity();

    UpstreamControlData update_control_data(DownstreamControlData downstream_control_data_);
};


/**
 * @brief Initializer helper for the Position Control Service.
 *        It is required the client to call this function before
 *        starting to perform position control.
 *
 * @param i_position_control Communication interface to the Position Control Service.
 */
void init_position_velocity_control(interface PositionVelocityCtrlInterface client i_position_control);

/**
 * @brief Service to perform a Position PID Control Loop on top of a Motor Control Service.
 *        You will need a Motor Control Stack running parallel to this Service,
 *        have a look at Motor Control Service for more information.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param position_ctrl_config Configuration for the Position Control Service.
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param i_position_control Array of communication interfaces to handle up to 3 different clients.
 */
void motion_control_service(int app_tile_usec, PosVelocityControlConfig &pos_velocity_control_config,
                    interface MotorcontrolInterface client i_motorcontrol,
                    interface PositionVelocityCtrlInterface server i_position_control[3],
                    client interface update_brake i_update_brake);
