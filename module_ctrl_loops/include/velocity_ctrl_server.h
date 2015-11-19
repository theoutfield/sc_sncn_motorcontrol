/**
 * @file velocity_ctrl_server.h
 * @brief Velocity Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
 */

#pragma once

#include <hall_server.h>
#include <qei_server.h>
#include <internal_config.h>
#include "control_loops_common.h"
#include "velocity_ctrl_client.h"
#include <commutation_server.h>

/**
 * @brief Velocity Control Loop
 *
 * @Input
 * @param velocity_ctrl_params struct defines the velocity control parameters
 * @param sensor_filter_par struct defines the filter parameters
 * @param hall_params struct defines the poles for hall sensor and gear-ratio
 * @param qei_params struct defines the resolution for qei sensor and gear-ratio
 * @param sensor_used specify the sensors to used via HALL/QEI defines
 *
 * @Input Channel
 * @param c_hall channel to receive position information from hall
 * @param c_qei channel to receive position information from qei
 * @param c_velocity_ctrl channel to receive/send velocity control information
 *
 * @Output Channel
 * @param c_commutation channel to send motor voltage input value
 *
 */
[[combinable]]
void velocity_control(ctrl_par & velocity_ctrl_params,
                        filter_par & sensor_filter_params,
                        hall_par &?hall_params,
                        qei_par &?qei_params,
                        int sensor_used,
                        interface HallInterface client i_hall,
                        interface QEIInterface client i_qei,
                        chanend c_velocity_ctrl,
                        interface CommutationInterface client commutation_interface);

