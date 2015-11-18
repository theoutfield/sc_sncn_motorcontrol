/**
 * @file comm.h
 * @brief Ctrlproto data struct client
 * @author Synapticon GmbH <support@synapticon.com>
*/

#pragma once

#include <hall_config.h>
#include <qei_config.h>
#include <internal_config.h>
#include <ctrlproto.h>
#include <commutation_client.h>
#include <control_loops_common.h>

/**
 * @brief Get target torque from Ethercat
 *
 * @return target torque from Ethercat in range [0 - mNm * Current Resolution]
 */
int get_target_torque(ctrl_proto_values_t InOut);

/**
 * @brief Get target velocity from Ethercat
 *
 * @return target velocity from Ethercat in rpm
 */
int get_target_velocity(ctrl_proto_values_t InOut);

/**
 * @brief Get target position from Ethercat
 *
 * @return target position from Ethercat in ticks
 */
int get_target_position(ctrl_proto_values_t InOut);

/**
 * @brief Send actual torque to Ethercat
 *
 * @param[in] actual_torque sent to Ethercat in range [0 - mNm * Current Resolution]
 */
void send_actual_torque(int actual_torque, ctrl_proto_values_t &InOut);

/**
 * @brief Send actual velocity to Ethercat
 *
 * @param[in] actual_velocity sent to Ethercat in rpm
 * @param[in] ctrl_proto_values_t
 */
void send_actual_velocity(int actual_velocity, ctrl_proto_values_t &InOut);

/**
 * @brief Send actual position to Ethercat
 *
 * @param[in] actual_position sent to Ethercat in ticks
 * @param[in] ctrl_proto_values_t
 */
void send_actual_position(int actual_position, ctrl_proto_values_t &InOut);

/**
 * @brief Update Hall sensor parameters from Ethercat
 *
 * @param hall_config struct defines the pole-pairs and gear ratio
 * @param coe_out
 */
void update_hall_config_ecat(HallConfig &hall_config, chanend coe_out);

/**
 * @brief Update QEI sensor parameters from Ethercat
*
 * @param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
*    gear-ratio used for the motor
*/
void update_qei_param_ecat(qei_par &qei_params, chanend coe_out);

void update_commutation_param_ecat(commutation_par &commutation_params, chanend coe_out);

/**
 * @brief Update cyclic synchronous torque parameters from Ethercat
*
 * @param cst_params struct defines the cyclic synchronous torque params
 * @param coe_out
*/
void update_cst_param_ecat(cst_par &cst_params, chanend coe_out);

/**
 * @brief Update cyclic synchronous velocity parameters from Ethercat
*
 * @param csv_params struct defines the cyclic synchronous velocity params
 * @param coe_out
*
*/
void update_csv_param_ecat(csv_par &csv_params, chanend coe_out);

/**
 * @brief Update cyclic synchronous position parameters from Ethercat
*
 * @param csp_params struct defines the cyclic synchronous position params
 * @param coe_out
*/
void update_csp_param_ecat(csp_par &csp_params, chanend coe_out);

/**
 * @brief Update profile torque parameters from Ethercat
*
 * @param pt_params struct defines the profile torque params
 * @param coe_out
*/
void update_pt_param_ecat(pt_par &pt_params, chanend coe_out);

/**
 * @brief Update profile velocity parameters from Ethercat
*
 * @param pv_params struct defines the profile velocity params
 * @param coe_out
*/
void update_pv_param_ecat(pv_par &pv_params, chanend coe_out);

/**
 * @brief Update profile position parameters from Ethercat
*
 * @param pp_params struct defines the profile position params
 * @param coe_out
*/
void update_pp_param_ecat(pp_par &pp_params, chanend coe_out);

/**
 * @brief Update torque control PID parameters from Ethercat
 *
 * @param torque_ctrl_params struct defines torque control PID params
 */
void update_torque_ctrl_param_ecat(ctrl_par &torque_ctrl_params, chanend coe_out);

/**
 * @brief Update velocity control PID parameters from Ethercat
 *
 * @param velocity_ctrl_params struct defines velocity control PID params
 */
void update_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend coe_out);

/**
 * @brief Update position control PID params from Ethercat
 *
 * @param position_ctrl_params struct defines position control PID params
 */
void update_position_ctrl_param_ecat(ctrl_par &position_ctrl_params, chanend coe_out);

/**
 * @brief Set commutation parameters from Ethercat communication loop
 *
 * @param[out] c_signal channel to signal Commutation loop
 * @param hall_config struct defines the pole-pairs and gear ratio
 * @param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 * @param nominal_speed defines nominal speed for the motor
 */
void set_commutation_param_ecat(chanend c_signal, HallConfig &hall_config, qei_par &qei_params, \
		commutation_par &commutation_params, int nominal_speed);

/**
 * @brief Set hall sensor parameters from Ethercat communication loop
 *
 * @param[out] c_hall channel to signal Hall acquisition loop
 *
 * @param hall_config struct defines the pole-pairs and gear ratio
 */
void set_hall_conifg_ecat(chanend c_hall, HallConfig &hall_config);

/**
 * @brief Set QEI sensor parameters from Ethercat communication loop
 *
 * @param[out] c_qei channel to signal QEI acquisition loop
 *
 * @param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void set_qei_param_ecat(chanend c_qei, qei_par &qei_params);

/**
 * @brief Initialize hall sensor parameters from Ethercat communication loop
 * 			(call before start-up of Hall acquisition loop)
 *
 * @param[out] c_hall channel to signal Hall acquisition loop
 *
 * @param hall_config struct defines the pole-pairs and gear ratio
 */
void hall_init_ecat(chanend c_hall, HallConfig &hall_config);

/**
 * @brief Initialize QEI sensor parameters from Ethercat communication loop
 *			(call before start-up of QEI acquisition loop)
 *
 * @param[out] c_qei channel to signal QEI acquisition loop
 *
 * @Input
 * @param qei_params struct defines the quadrature encoder (QEI) resolution, sensor type and
 * 	 gear-ratio used for the motor
 */
void qei_init_ecat(chanend c_qei, qei_par &qei_params);

