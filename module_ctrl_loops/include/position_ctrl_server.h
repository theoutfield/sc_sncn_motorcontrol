/**
 * @file  position_ctrl_server.h
 * @brief Position Control Loop Server Implementation
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <control_loops_common.h>
#include <qei_server.h>
#include "control_loops_common.h"
#include "position_ctrl_client.h"
#include <commutation_server.h>
/**
 * @brief Position Control Loop
 *  Implements PID controller for position using Hall or QEI sensors.
 *  Note: The Server must be placed on CORES 0/1/2 only.
 *
 * @Input
 * @param position_ctrl_params struct defines the position control parameters
 * @param hall_params struct defines the poles for hall sensor and gear-ratio
 * @param qei_params struct defines the resolution for qei sensor and gear-ratio
 * @param sensor_used specify the sensors to used via HALL/QEI defines
 *
 * @Input Channel
 * @param c_hall channel to receive position information from hall
 * @param c_qei channel to receive position information from qei
 * @param c_position_ctrl channel to receive/send position control information
 *
 * @Output Channel
 * @param c_commutation channel to send motor voltage input value
 *
 */
void position_control(ctrl_par & position_ctrl_params, hall_par & hall_params, qei_par & qei_params, int sensor_used,
                    interface HallInterface client i_hall, interface QEIInterface client i_qei, chanend c_position_ctrl,
                    interface CommutationInterface client commutation_interface);

