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
#define MIN_TORQUE_CONTROL_LOOP_PERIOD 100 //us
/**
 * @brief Lorem ipsum...
 */
interface TorqueControlInterface{
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
    int check_torque_ctrl_state();
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_torque();
    /**
     * @brief Lorem ipsum...
     *
     * @param in_torque Lorem ipsum...
     */
    void set_torque(int in_torque);
    /**
     * @brief Lorem ipsum...
     *
     * @return Lorem ipsum...
     */
    int get_set_torque();
    /**
     * @brief Lorem ipsum...
     *
     * @param torque_ctrl_params Lorem ipsum...
     */
    void set_torque_ctrl_param(ControlConfig torque_ctrl_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param hall_config Lorem ipsum...
     */
    void set_torque_ctrl_hall_param(HallConfig hall_config);
    /**
     * @brief Lorem ipsum...
     *
     * @param qei_params Lorem ipsum...
     */
    void set_torque_ctrl_qei_param(QEIConfig qei_params);
    /**
     * @brief Lorem ipsum...
     *
     * @param sensor_used Lorem ipsum...
     */
    void set_torque_sensor(int sensor_used);
    /**
     * @brief Lorem ipsum...
     */
    void enable_torque_ctrl();
    /**
     * @brief Lorem ipsum...
     */
    void shutdown_torque_ctrl();

};

/**
 * @brief Initialise Torque Control Loop
 *
 * @Input Channel
 * @param i_torque_control Lorem ipsum...
 */
int init_torque_control(interface TorqueControlInterface client i_torque_control);

/**
 * @brief Torque Limiter
 *
 * @Input
 * @param torque is the input torque to be limited in range
 * @param max_torque_limit is the max torque that can be reached
 *
 * @Output
 * @return torque in the range [-max_torque_limit to max_torque_limit] (mNm * Current resolution)
 */
int torque_limit(int torque, int max_torque_limit);

/**
 * @brief Set new target torque for torque control (advanced function)
 *
 * @Input Channel
 * @param c_torque_ctrl channel to signal new target torque
 *
 * @Input
 * @param cst_params struct defines the motor parameters and torque limits
 * @param target_torque is the new target torque
 * @param torque_offset defines offset in torque
 */
void set_torque_cst(ProfilerConfig & cst_params, int target_torque, int torque_offset, interface TorqueControlInterface client i_torque_control);

/**
 * @brief Torque Control Loop
 *
 * @param torque_ctrl_params struct defines the torque control parameters
 * @param adc_if Lorem ipsum...
 * @param commutation_interface Lorem ipsum...
 * @param i_hall Lorem ipsum...
 * @param i_qei Lorem ipsum...
 * @param i_torque_control[3] Lorem ipsum...
 *
 */
void torque_control_service(ControlConfig &torque_ctrl_params,
                            interface ADCInterface client adc_if,
                            interface MotorcontrolInterface client commutation_interface,
                            interface HallInterface client i_hall,
                            interface QEIInterface client ?i_qei,
                            interface TorqueControlInterface server i_torque_control[3]);


/**
 * @brief Enable ADC for current/torque calculations
 *
 * @Input Channel
 * @param c_current channel to receive a filtered current information from the current sensor
 *
 */
void enable_adc(chanend c_current);
