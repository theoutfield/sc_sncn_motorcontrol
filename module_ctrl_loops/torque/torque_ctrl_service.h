/**
 * @file  torque_ctrl_server.h
 * @brief Torque Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <control_loops_common.h>
#include <motorcontrol_service.h>
#include <hall_service.h>
#include <qei_service.h>
#include <adc_service.h>

#define FILTER_LENGTH_TORQUE 80

/**
 * @brief Minimum period for the Torque Control Loop is 100 us.
 */
#define MIN_TORQUE_CONTROL_LOOP_PERIOD 100 //us

/**
 * @brief Interface type to communicate with the Torque Control Service.
 */
interface TorqueControlInterface{

    /**
     * @brief Enables the Service operation.
     */
    void enable_torque_ctrl();

    /**
     * @brief Disables the Service operation.
     */
    void disable_torque_ctrl();

    /**
     * @brief Setter for new target torque in the controller.
     *        The real torque equivalent to this input depends
     *        on the SOMANET IFM DC device that you use and
     *        the torque constant of your motor.
     *
     * @param in_torque New target torque [-8191:8192].
     */
    void set_torque(int in_torque);

    /**
     * @brief Getter for the current torque applied in your motor.
     *        The real torque equivalent to the return value depends
     *        on the SOMANET IFM DC device that you use and
     *        the torque constant of your motor.
     *
     * @return Current torque [-8191:8192].
     */
    int get_torque();

    /**
     * @brief Getter for the current target torque in the controller.
     *
     * @param Current target torque [-8191:8192].
     */
    int get_target_torque();

    /**
     * @brief Getter for current configuration used by the Service.
     *
     * @return Current Service configuration.
     */
    ControlConfig get_torque_control_config();

    /**
     * @brief Setter for new configuration in the Service.
     *
     * @param in_config New Service configuration.
     */
    void set_torque_control_config(ControlConfig torque_ctrl_config);

    /**
     * @brief Allows you to change the position sensor on runtime.
     *
     * @param sensor New sensor [HALL_SENSOR, QEI_SENSOR].
     */
    void set_torque_sensor(int sensor_used);

    /**
     * @brief Setter for new configuration in the Hall Sensor Service.
     *
     * @param in_config New Hall Sensor Service configuration.
     */
    void set_hall_config(HallConfig in_config);

    /**
     * @brief Setter for new configuration in the Encoder Service.
     *
     * @param in_config New Encoder Service configuration.
     */
    void set_qei_config(QEIConfig in_config);

    /**
     * @brief Getter for the current state of the Service.
     *
     * @return 0 - not initialized.
     *         1 - initialized.
     */
    int check_busy();
};

/**
 * @brief Initializer helper for the Torque Control Service.
 *        It is required the client to call this function before
 *        starting to perform torque control.
 *
 * @param i_torque_control Communication interface to the Torque Control Service.
 */
void init_torque_control(interface TorqueControlInterface client i_torque_control);

/**
 * @brief Torque limiter helper.
 *
 * @param torque The input torque to be limited in range.
 * @param max_torque_limit Max torque that can be reached.
 *
 * @return Torque in the range [-max_torque_limit:max_torque_limit].
 */
int torque_limit(int torque, int max_torque_limit);

/**
 * @brief Service to perform a Torque PID Control Loop on top of a Motor Control Service.
 *        You will need a motor control stack running parallely to this Service,
 *        have a look at Motor Control Service for more information.
 *        The motor control stack must contain an ADC Service triggered by the PWM Service.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param torque_ctrl_config Configuration for the Torque Control Service.
 * @param adc_if Communication interface to the ADC Service.
 * @param i_hall [[Nullable]] Communication interface to the Hall Sensor Service (if applicable).
 * @param i_qei [[Nullable]] Communication interface to the Encoder Service (if applicable).
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param Array of communication interfaces to handle up to 3 different clients.
 */
void torque_control_service(ControlConfig &torque_ctrl_config,
                            interface ADCInterface client i_adc,
                            interface HallInterface client i_hall,
                            interface QEIInterface client ?i_qei,
                            interface MotorcontrolInterface client i_motorcontrol,
                            interface TorqueControlInterface server i_torque_control[3]);
