/**
 * @file  torque_ctrl_server.h
 * @brief Torque Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <commutation_service.h>
#include <hall_service.h>
#include <qei_service.h>
#include <internal_config.h>
#include "control_loops_common.h"
#include "torque_ctrl_client.h"
#include <adc_service.h>


/**
 * @brief Torque Control Loop
 *
 * @Input
 * @param torque_ctrl_params struct defines the torque control parameters
 * @param hall_config struct defines the poles for hall sensor and gear-ratio
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
void torque_control(ctrl_par & torque_ctrl_params, HallConfig & hall_config, qei_par & qei_params,
                    int sensor_used, interface ADCInterface client adc_if, interface CommutationInterface client commutation_interface,
                    interface HallInterface client i_hall, interface QEIInterface client i_qei, chanend c_torque_ctrl);

/**
 * @brief Enable ADC for current/torque calculations
 *
 * @Input Channel
 * @param c_current channel to receive a filtered current information from the current sensor
 *
 */
void enable_adc(chanend c_current);
