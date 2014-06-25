/**
 * @file drive_config.h
 * @brief Motor Drive defines and configurations
 * @author Pavan Kanajar <pkanajar@synapticon.com>
*/

#pragma once

#include <stdint.h>


typedef int bool;
enum { false, true };

typedef struct S_Check_list {
    bool ready;
    bool switch_on;
    bool operation_enable;
    bool mode_op;
    bool fault;

    bool _commutation_init;
    bool _hall_init;
    bool _qei_init;
    bool _adc_init;
    bool _torque_init;
    bool _velocity_init;
    bool _position_init;
} check_list;


bool __check_bdc_init(chanend c_signal);
/**
 * @brief Check commutation initialization
 *
 * @Output
 * @return init state of the commutation loop
 */
bool __check_commutation_init(chanend c_signal);

/**
 * @brief Check hall initialization
 *
 * @Output
 * @return init state of the hall loop
 */
bool __check_hall_init(chanend c_hall);

/**
 * @brief Check qei initialization
 *
 * @Output
 * @return init state of the qei loop
 */
bool __check_qei_init(chanend c_qei);

/**
 * @brief Check adc initialization
 *
 * @Output
 * @return init state of the adc loop
 */
bool __check_adc_init();

/**
 * @brief Check torque control initialization
 *
 * @Output
 * @return init state of the torque control loop
 */
bool __check_torque_init(chanend c_torque_ctrl);

/**
 * @brief Check velocity control initialization
 *
 * @Output
 * @return init state of the velocity control loop
 */
bool __check_velocity_init(chanend c_velocity_ctrl);

/**
 * @brief Check position control initialization
 *
 * @Output
 * @return init state of the position control loop
 */
bool __check_position_init(chanend c_position_ctrl);

int init_state(void);

/**
 * @brief Initialize checklist params
 *
 * @Output
 * @return check_list initialised checklist parameters
 */
check_list init_checklist(void);

/**
 * @brief Update Checklist
 *
 * @Input channel
 * @param c_commutation for communicating with the commutation server
 * @param c_hall for communicating with the hall server
 * @param c_qei for communicating with the qei server
 * @param c_adc for communicating with the adc server
 * @param c_torque_ctrl for communicating with the torque control server
 * @param c_velocity_ctrl for communicating with the velocity control server
 * @param c_position_ctrl for communicating with the position control server
 *
 * @Input
 * @param mode sets mode of operation
 *
 * @Output
 * @return check_list_param updated checklist parameters
 */
void update_checklist(check_list & check_list_param, int mode, chanend c_commutation, chanend c_hall, chanend c_qei,
                      chanend c_adc, chanend c_torque_ctrl, chanend c_velocity_ctrl, chanend c_position_ctrl);

int16_t update_statusword(int current_status, int state_reached, int ack, int q_active, int shutdown_ack);

int get_next_state(int in_state, check_list &checklist, int controlword);

int read_controlword_switch_on(int control_word);

int read_controlword_quick_stop(int control_word);

int read_controlword_enable_op(int control_word);

int read_controlword_fault_reset(int control_word);

