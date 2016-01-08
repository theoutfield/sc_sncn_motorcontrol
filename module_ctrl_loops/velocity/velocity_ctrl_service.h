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

#define DEFAULT_FILTER_LENGTH 8

/**
 * @brief Interface type to communicate with the Velocity Control Service.
 */
interface VelocityControlInterface{

    /**
     * @brief Enables the Service operation.
     */
    void enable_velocity_ctrl();

    /**
     * @brief Disables the Service operation.
     */
    void disable_velocity_ctrl();

    /**
     * @brief Setter for new target velocity in the controller.
     *
     * @param target_velocity New target velocity [RPMs].
     */
    void set_velocity(int target_velocity);

    /**
     * @brief Getter for the current velocity of your motor.
     *
     * @return Current velocity [RPMs].
     */
    int get_velocity();

    /**
     * @brief Getter for the current target velocity in the controller.
     *
     * @param Current target velocity [RPMs].
     */
    int get_target_velocity();

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    ControlConfig get_velocity_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_velocity_control_config(ControlConfig velocity_ctrl_params);

    /**
     * @brief Allows you to change the sensor for velocity control on runtime.
     *
     * @param sensor New sensor [HALL_SENSOR, QEI_SENSOR].
     */
    void set_velocity_sensor(int sensor_used);

    /**
     * @brief Allows you to change the length of the filter applied over the measured velocity.
     *
     * @param in_length New length [0:128].
     */
    void set_velocity_filter(int in_length);

    /**
     * @brief Setter for new configuration in the Hall Sensor Service.
     *
     * @param in_config New Hall Sensor Service configuration.
     */
    void set_hall_config(HallConfig hall_config);

    /**
     * @brief Setter for new configuration in the Encoder Service.
     *
     * @param in_config New Encoder Service configuration.
     */
    void set_qei_config(QEIConfig qei_params);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
    int check_busy();
};

/**
 * @brief Initializer helper for the Velocity Control Service.
 *        It is required the client to call this function before
 *        starting to perform position control.
 *
 * @param i_velocity_control Communication interface to the Velocity Control Service.
 */
void init_velocity_control(interface VelocityControlInterface client i_velocity_control);

/**
 * @brief Velocity limiter helper.
 *
 * @param velocity The input velocity to be limited in range.
 * @param max_speed Max speed that can be reached.
 *
 * @return Velocity in the range [-max_speed:max_speed].
 */
int max_speed_limit(int velocity, int max_speed);

/**
 * @brief Service to perform a Velocity PID Control Loop on top of a Motor Control Service.
 *        You will need a Motor Control Stack running parallely to this Service,
 *        have a look at Motor Control Service for more information.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param velocity_ctrl_config Configuration for the Velocity Control Service.
 * @param i_hall [[Nullable]] Communication interface to the Hall Sensor Service (if applicable).
 * @param i_qei [[Nullable]] Communication interface to the Encoder Service (if applicable).
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param i_velocity_control[3] Array of communication interfaces to handle up to 3 different clients.
 */
[[combinable]]
void velocity_control_service(ControlConfig & velocity_ctrl_config,
                        interface HallInterface client ?i_hall,
                        interface QEIInterface client ?i_qei,
                        interface MotorcontrolInterface client i_motorcontrol,
                        interface VelocityControlInterface server i_velocity_control[3]);
