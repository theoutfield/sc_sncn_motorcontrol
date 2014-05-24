
/**
 * \file  position_ctrl_server.h
 * \brief Position Control Loop Server Implementation
 * \author Pavan Kanajar <pkanajar@synapticon.com>
*/

#include <internal_config.h>
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop_client.h"

/**
 * \brief Position Control Loop
 *		Implements PID controller for position using Hall or QEI sensors.
 *		Note: The Server must be placed on CORES 0/1/2 only.
 *
 *  Input
 * \param position_ctrl_params struct defines the position control parameters
 * \param hall_params struct defines the poles for hall sensor and gear-ratio
 * \param qei_params struct defines the resolution for qei sensor and gear-ratio
 * \param sensor_used specify the sensors to used via HALL/QEI defines
 *
 *  Input Channel
 * \channel c_hall channel to receive position information from hall
 * \channel c_qei channel to receive position information from qei
 * \channel c_position_ctrl channel to receive/send position control information
 *
 *  Output Channel
 * \channel c_commutation channel to send motor voltage input value
 *
 */
void position_control(ctrl_par &position_ctrl_params, hall_par &hall_params, qei_par &qei_params, int sensor_used, \
		              chanend c_hall, chanend c_qei, chanend c_position_ctrl, chanend c_commutation);

