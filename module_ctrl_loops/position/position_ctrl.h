#include <dc_motor_config.h>
#include <internal_config.h>
#include "refclk.h"
#include "qei_client.h"
#include "hall_client.h"
#include "comm_loop.h"


/**
 * \brief Initialise Position Control Loop
 *  Input Channel
 * \channel c_position_ctrl channel to signal initialisation
 */
int init_position_control(chanend c_position_ctrl);


/**
 * \brief Position Limiter
 *
 *  Input
 * \param position is the input position to be limited in range
 * \param max_position_limit is the max position that can be reached
 * \param min_position_limit is the min position that can be reached
 *
 *  Output
 * \return position in the range [min_position_limit - max_position_limit]
 */
int position_limit(int position, int max_position_limit, int min_position_limit);


/**
 * \brief Set new target position for position control
 *
 *  Input Channel
 * \channel c_position_ctrl channel to signal new target position
 *
 *  Input
 * \param target_position is the new target position
 */
void set_position(int target_position, chanend c_position_ctrl);

/**
 * \brief Get actual position from position control
 *
 *  Output Channel
 * \channel c_position_ctrl channel to receive actual position
 *
 *  Output
 * \return actual position from position control
 */
int get_position(chanend c_position_ctrl);

/**
 * \brief Set Position Control PID Parameters
 *
 *  Input
 * \param position_ctrl_params struct defines the position control PID parameters
 */
void set_position_ctrl_param(ctrl_par &position_ctrl_params, chanend c_position_ctrl);

/**
 * \brief initialize hall sensor parameters for Position Control
 *
 * \param hall_params struct defines the pole-pairs and gear ratio
 */
void set_position_ctrl_hall_param(hall_par &hall_params, chanend c_position_ctrl);

/**
 * \brief initialize QEI sensor for Position Control
 *
 * \param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_position_ctrl_qei_param(qei_par &qei_params, chanend c_position_ctrl);

/**
 * \brief Sets the sensor used for Position Control
 *
 * \param sensor_used defines the sensor to be used (HALL/QEI) for Position Control
 */
void set_position_sensor(int sensor_used, chanend c_position_ctrl);

/**
 * \brief Enables Position Control mode operation
 *
 * \channel c_position_ctrl channel to signal enable position control
 */
void enable_position_ctrl(chanend c_position_ctrl);

/**
 * \brief Shutdown Position Control mode operation
 *
 * \channel c_position_ctrl channel to signal shutdown of position control
 */
void shutdown_position_ctrl(chanend c_position_ctrl);

/**
 * \brief Set new target position for position control
 *
 *  Input Channel
 * \channel c_position_ctrl channel to signal new target position
 *
 *  Input
 * \param csp_param struct defines the motor parameters and position limits
 * \param target_position is the new target position
 * \param position_offset defines offset in position
 * \param velocity_offset defines offset in velocity
 * \param torque_offset defines offset in torque
 */
void set_position_csp(csp_par &csp_params, int target_position, int position_offset, int velocity_offset,\
		              int torque_offset, chanend c_position_ctrl);

/**
 * \brief Position Control Loop
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
