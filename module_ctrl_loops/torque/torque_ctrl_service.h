/**
 * @file  torque_ctrl_service.h
 * @brief Torque Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <control_loops_common.h>
#include <motorcontrol_service.h>
#include <hall_service.h>
#include <qei_service.h>
#include <biss_service.h>
#include <ams_service.h>
#include <adc_service.h>

#define FILTER_LENGTH_TORQUE 20
#define FILTER_LENGTH_ADC 80

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
     * @param in_torque New target torque [-8191:8192]. (8192 is equivalent to the max current your SOMANET IFM DC device can handle: DC100: 5A, DC300: 20A, DC1K 50A).
     */
    void set_torque(int in_torque);

    /**
     * @brief Getter for the current torque applied in your motor.
     *        The real torque equivalent to the return value depends
     *        on the SOMANET IFM DC device that you use and
     *        the torque constant of your motor.
     *
     * @return Current torque [-8191:8192]. (8192 is equivalent to the max current your SOMANET IFM DC device can handle: DC100: 5A, DC300: 20A, DC1K 50A).
     */
    int get_torque();

    /**
     * @brief Getter for the current target torque in the controller.
     *
     * @return Current target torque [-8191:8192]. (8192 is equivalent to the max current your SOMANET IFM DC device can handle: DC100: 5A, DC300: 20A, DC1K 50A).
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
    void set_torque_control_config(ControlConfig in_config);

    /**
     * @brief Allows you to change the position sensor on runtime.
     *
     * @param sensor_used New sensor [HALL_SENSOR, QEI_SENSOR].
     */
    void set_torque_sensor(int sensor_used);

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
 *        You will need a motor control stack running parallel to this Service,
 *        have a look at Motor Control Service for more information.
 *        The motor control stack must contain an ADC Service triggered by the PWM Service.
 *
 *  Note: It is important to allocate this service in a different tile from the remaining Motor Control stack.
 *
 * @param torque_ctrl_config Configuration for the Torque Control Service.
 * @param i_adc Communication interface to the ADC Service.
 * @param i_hall [[Nullable]] Communication interface to the Hall Sensor Service (if applicable).
 * @param i_qei [[Nullable]] Communication interface to the Encoder Service (if applicable).
 * @param i_biss [[Nullable]] Communication interface to the BiSSEncoder Service (if applicable).
 * @param i_ams [[Nullable]] Communication interface to the AMSEncoder Service (if applicable).
 * @param i_motorcontrol Communication interface to the Motor Control Service.
 * @param i_torque_control Array of communication interfaces to handle up to 3 different clients.
 */
void torque_control_service(ControlConfig &torque_ctrl_config,
                            interface ADCInterface client i_adc,
                            interface HallInterface client ?i_hall,
                            interface QEIInterface client ?i_qei,
                            interface BISSInterface client ?i_biss,
                            interface AMSInterface client ?i_ams,
                            interface MotorcontrolInterface client i_motorcontrol,
                            interface TorqueControlInterface server i_torque_control[3]);
