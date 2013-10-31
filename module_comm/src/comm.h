#ifndef COMM_H_
#define COMM_H_
#pragma once
#include <dc_motor_config.h>
#include <refclk.h>
#include <ctrlproto.h>
#include <qei_client.h>
#include <internal_config.h>

int get_target_torque(ctrl_proto_values_t InOut);

int get_target_velocity(ctrl_proto_values_t InOut);

int get_target_position(ctrl_proto_values_t InOut);

void send_actual_torque(int actual_torque, ctrl_proto_values_t &InOut);

void send_actual_velocity(int actual_velocity, ctrl_proto_values_t &InOut);

void send_actual_position(int actual_position, ctrl_proto_values_t &InOut);

void update_hall_param_ecat(hall_par &hall_params, chanend coe_out);

void update_qei_param_ecat(qei_par &qei_params, chanend coe_out);

void update_cst_param_ecat(cst_par &cst_params, chanend coe_out);

void update_csv_param_ecat(csv_par &csv_params, chanend coe_out);

void update_csp_param_ecat(csp_par &csp_params, chanend coe_out);

void update_pt_param_ecat(pt_par &pt_params, chanend coe_out);

void update_pv_param_ecat(pv_par &pv_params, chanend coe_out);

void update_pp_param_ecat(pp_par &pp_params, chanend coe_out);

void update_torque_ctrl_param_ecat(ctrl_par &torque_ctrl_params, chanend coe_out);

void update_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend coe_out);

void update_position_ctrl_param_ecat(ctrl_par &position_ctrl_params, chanend coe_out);

void set_commutation_param_ecat(chanend c_signal, hall_par &hall_params); /*client*/
void comm_init_ecat(chanend c_signal, hall_par &hall_params);

void set_hall_param_ecat(chanend c_hall, hall_par &hall_params); /*client*/
void hall_init_ecat(chanend c_hall, hall_par &hall_params);

void set_qei_param_ecat(chanend c_qei, qei_par &qei_params);	/*client*/
void qei_init_ecat(chanend c_qei, qei_par &qei_params);

#endif /* COMM_H_ */
