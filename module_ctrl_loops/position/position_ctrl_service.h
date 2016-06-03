/**
 * @file  position_ctrl_service.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <motorcontrol_service.h>
#include <control_loops_common.h>

/**
 * @brief Interface type to communicate with the Position Control Service.
 */
interface PositionControlInterface{



            void enable_position_ctrl();
            void disable_position_ctrl();
            void set_position(int in_target_position);
            void set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
            void set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
            void set_position_limits(int position_min_limit, int position_max_limit);



            void enable_velocity_ctrl();
            void disable_velocity_ctrl();
            void set_velocity(int in_target_velocity);
            void set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
            void set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_itegral_limit, int int16_cmd_limit);
            void set_velocity_limits(int velocity_min_limit, int velocity_max_limit);



            void enable_torque_ctrl();
            void disable_torque_ctrl();
            void set_torque(int in_target_torque);
            void set_torque_limits(int torque_min_limit, int torque_max_limit);



            int get_position();


            int get_velocity();



    /**
     * @brief Enables the Service operation.
     */
//    void enable_position_ctrl();

    /**
     * @brief Disables the Service operation.
     */
//    void disable_position_ctrl();

    /**
     * @brief Setter for new target position in the controller.
     *
     * @param target_position New target position [INT_MIN:INT_MAX].
     */
//    void set_position(int target_position);
//
//    void set_position_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//
//    void set_position_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_integral_limit, int int32_cmd_limit);
//
//    void set_velocity_pid_coefficients(int int8_Kp, int int8_Ki, int int8_Kd);
//
//    void set_velocity_pid_limits(int int16_P_error_limit, int int16_I_error_limit, int int16_integral_limit, int int32_cmd_limit);

    /**
     * @brief Getter for the current position of your motor.
     *
     * @return Current position [INT_MIN:INT_MAX].
     */
//    int get_position();

    /**
     * @brief Getter for the current target position in the controller.
     *
     * @return Current target position [INT_MIN:INT_MAX].
     */
//    int get_target_position();

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
//    ControlConfig get_position_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
//    void set_position_control_config(ControlConfig in_config);

    /**
     * @brief Allows you to change the position control sensor on runtime.
     *
     * @param sensor_used New sensor [HALL_SENSOR, QEI_SENSOR].
     */
//    void set_position_sensor(int sensor_used);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
//    int check_busy();

//    void set_torque_limit(int in_torque_limit);
};



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
void position_control_service(ControlConfig & position_ctrl_config,
                    interface MotorcontrolInterface client i_motorcontrol,
                    interface PositionControlInterface server i_position_control[3]);
