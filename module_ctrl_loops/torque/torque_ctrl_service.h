/**
 * @file  torque_ctrl_server.h
 * @brief Torque Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <motorcontrol_service.h>
#include <hall_service.h>
#include <qei_service.h>
#include <internal_config.h>
#include "control_loops_common.h"
#include <adc_service.h>


interface TorqueControlInterface{

    int check_busy();
    int check_torque_ctrl_state();
    int get_torque();
    void set_torque(int in_torque);
    void set_torque_ctrl_param(ControlConfig torque_ctrl_params);
    void set_torque_ctrl_hall_param(HallConfig hall_config);
    void set_torque_ctrl_qei_param(QEIConfig qei_params);
    void set_torque_sensor(int sensor_used);
    void enable_torque_ctrl();
    void shutdown_torque_ctrl();

};

/**
 * @brief Initialise Torque Control Loop
 *
 * @Input Channel
 * @param c_torque_ctrl channel to signal initialisation
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
 * @param cst_param struct defines the motor parameters and torque limits
 * @param target_torque is the new target torque
 * @param torque_offset defines offset in torque
 */
void set_torque_cst(cst_par & cst_params, int target_torque, int torque_offset, interface TorqueControlInterface client i_torque_control);

/**
 * @brief Torque Control Loop
 *
 * @Input
 * @param torque_ctrl_params struct defines the torque control parameters
 * @param hall_params struct defines the poles for hall sensor and gear-ratio
 * @param qei_params struct defines the resolution for qei sensor and gear-ratio
 * @param sensor_select specify the sensor to use via HALL/QEI defines
 *
 * @Input Channel
 * @param c_adc channel to receive torque information from current sensor
 * @param c_hall channel to receive position information from hall
 * @param c_qei channel to receive position information from qei
 * @param c_torque channel to receive/send torque control information
 *
 * @Output Channel
 * @param c_commutation channel to send motor voltage input value
 *
 */
void torque_control_service(ControlConfig &torque_ctrl_params,
                            interface ADCInterface client adc_if,
                            interface MotorcontrolInterface client commutation_interface,
                            interface HallInterface client i_hall,
                            interface QEIInterface client ?i_qei,
                            interface TorqueControlInterface server i_torque_control);


/**
 * @brief Enable ADC for current/torque calculations
 *
 * @Input Channel
 * @param c_current channel to receive a filtered current information from the current sensor
 *
 */
void enable_adc(chanend c_current);
