/**
 * @file  position_ctrl_service.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <control_loops_common.h>
#include <motor_control_interfaces.h>
#include <advanced_motor_control.h>

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

    void enable_velocity_ctrl(int velocity_control_mode_);
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
//    void set_torque(int in_target_torque);
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
 * @param i_hall [[Nullable]] Communication interface to the Hall Sensor Service (if applicable).
 * @param i_qei [[Nullable]] Communication interface to the Incremental Encoder Service (if applicable).
 * @param i_biss [[Nullable]] Communication interface to the BiSSEncoder Service (if applicable).
 * @param i_ams [[Nullable]] Communication interface to the AMSEncoder Service (if applicable).
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param i_position_control Array of communication interfaces to handle up to 3 different clients.
 */
void position_velocity_control_service(PosVelocityControlConfig &pos_velocity_control_config,
                    interface MotorcontrolInterface client i_motorcontrol,
                    interface PositionVelocityCtrlInterface server i_position_control[3]);
