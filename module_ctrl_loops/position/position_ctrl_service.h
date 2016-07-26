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

    /**
     * @brief Enables the Service operation.
     */
    void enable_position_ctrl();

    /**
     * @brief Disables the Service operation.
     */
    void disable_position_ctrl();

    /**
     * @brief Setter for new target position in the controller.
     *
     * @param target_position New target position [INT_MIN:INT_MAX].
     */
    void set_position(int target_position);

    /**
     * @brief Getter for the current position of your motor.
     *
     * @return Current position [INT_MIN:INT_MAX].
     */
    int get_position();

    /**
     * @brief Getter for the current target position in the controller.
     *
     * @return Current target position [INT_MIN:INT_MAX].
     */
    int get_target_position();

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    ControlConfig get_position_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_position_control_config(ControlConfig in_config);

    /**
     * @brief Allows you to change the position control sensor on runtime.
     *
     * @param sensor_used New sensor [HALL_SENSOR, QEI_SENSOR].
     */
    void set_position_sensor(int sensor_used);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
    int check_busy();

    void set_torque_limit(int in_torque_limit);

    {int, int, int} set_pid_values(int Kp_in, int Ki_in, int Kd_in);
};

/**
 * @brief Initializer helper for the Position Control Service.
 *        It is required the client to call this function before
 *        starting to perform position control.
 *
 * @param i_position_control Communication interface to the Position Control Service.
 */
void init_position_control(interface PositionControlInterface client i_position_control);

/**
 * @brief Position limiter helper.
 *
 * @param position The input position to be limited in range.
 * @param max_position_limit Upper limit that can be reached.
 * @param min_position_limit Lower limit that can be reached.
 *
 * @return position in the range [min_position_limit:max_position_limit].
 */
int position_limit(int position, int max_position_limit, int min_position_limit);

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
                    interface HallInterface client ?i_hall,
                    interface QEIInterface client ?i_qei,
                    interface BISSInterface client ?i_biss,
                    interface AMSInterface client ?i_ams,
                    interface CONTELECInterface client ?i_contelec,
                    interface MotorcontrolInterface client i_motorcontrol,
                    interface PositionControlInterface server i_position_control[3]);
