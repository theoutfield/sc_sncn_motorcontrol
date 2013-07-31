/**
 * drive_sys.xc
 *
 *  Created on: Jul 12, 2013
 *      Author: pkanajar
 */

#include <drive_config.h>

int read_controlword_switch_on(int control_word) {
	return (control_word & SWITCH_ON_CONTROL);
}

int read_controlword_quick_stop(int control_word) {
	return (~((control_word & QUICK_STOP_CONTROL) >> 2) & 0x1);
}

int read_controlword_enable_op(int control_word) {
	return (control_word & ENABLE_OPERATION_CONTROL) >> 3;
}

int read_controlword_fault_reset(int control_word) {
	return (control_word & FAULT_RESET_CONTROL) >> 7;
}

void update_checklist(check_list *check_list_param) {
	bool check;
	bool skip = false;
	int mode;
	check = check_list_param->_adc_init & check_list_param->_commutation_init // & check_list_param->_fault
			& check_list_param->_hall_init & check_list_param->_qei_init;
	switch(check)
	{
		case false:
			if(~check_list_param->_commutation_init)
			{
				check_list_param->_commutation_init = __check_commutation_init();
				skip = true;
			}
			if(~skip & ~check_list_param->_adc_init)
			{
				check_list_param->_adc_init = __check_adc_init();
			}
			if(~skip & ~check_list_param->_hall_init)
			{
				check_list_param->_hall_init = __check_hall_init();
			}
			if(~skip & ~check_list_param->_qei_init)
			{
				check_list_param->_qei_init = __check_qei_init();
			}
			break;
		case true:
			if(~check_list_param->_torque_init & mode == 1)
			{
				check_list_param->_torque_init = __check_torque_init();
				skip = true;
			}
			if(~skip)
			{
				if(~check_list_param->_velocity_init & mode == 2)
				{
					check_list_param->_velocity_init = __check_velocity_init();
					break;
				}
				if(~check_list_param->_position_init & mode == 3)
				{
					check_list_param->_position_init = __check_position_init();
				}
			}

			break;
	}


}

int init_state(void) {
	return 1;
}

/**
 *
 */
int update_statusword(int current_status, int state_reached) {
	int status_word;

	switch (state_reached) {
	case 1:
		status_word = current_status & ~READY_TO_SWITCH_ON_STATE
				& ~SWITCHED_ON_STATE & ~OPERATION_ENABLED_STATE
				& ~VOLTAGE_ENABLED_STATE;
		break;

	case 7:
		status_word = (current_status & ~OPERATION_ENABLED_STATE
				& ~SWITCHED_ON_STATE & ~VOLTAGE_ENABLED_STATE
				& ~SWITCH_ON_DISABLED_STATE) | READY_TO_SWITCH_ON_STATE;
		break;

	case 2:
		status_word = (current_status & READY_TO_SWITCH_ON_STATE
				& ~OPERATION_ENABLED_STATE & ~SWITCHED_ON_STATE
				& ~VOLTAGE_ENABLED_STATE) | SWITCH_ON_DISABLED_STATE;
		break;

	case 3:
		status_word = (current_status & ~SWITCH_ON_DISABLED_STATE
				& ~OPERATION_ENABLED_STATE) | SWITCHED_ON_STATE
				| VOLTAGE_ENABLED_STATE;
		break;

	case 4:
		status_word = current_status | OPERATION_ENABLED_STATE;
		break;

	case 5:
		status_word = current_status | FAULT_STATE;
		break;

	case 6:
		status_word = current_status | QUICK_STOP_STATE;
		break;

	}
	return status_word;
}

int get_next_values(int in_state, int check_init, int ctrl_input, int fault) {
	int out_state;

	switch (in_state) {
	case 1:
		if (fault == 1)
			out_state = 5;
		else if (check_init == 0)
			out_state = 2;
		else if (check_init == 1)
			out_state = 7;
		break;

	case 2:
		if (fault == 1)
			out_state = 5;
		else if (check_init == 0)
			out_state = 1;
		else if (check_init == 1)
			out_state = 7;
		break;

	case 7:
		if (fault == 1)
			out_state = 5;
		else if (check_init == 0)
			out_state = 7;
		else if (check_init == 1)
			if (ctrl_input == 0)
				out_state = 7;
			else if (ctrl_input == 1)
				out_state = 3;
		break;

	case 3:
		if (fault == 1)
			out_state = 5;
		else if (check_init == 0)
			out_state = 3;
		else if (check_init == 1)
			if (ctrl_input == 0)
				out_state = 3;
			else if (ctrl_input == 1)
				out_state = 4;
		break;

	case 4:
		if (fault == 1)
			out_state = 5;
		else if (ctrl_input == 0)
			out_state = 4;
		else if (ctrl_input == 1) /*quick stop*/
			out_state = 6;
		break;

	case 5:
		if (ctrl_input == 0)
			out_state = 5;
		else if (ctrl_input == 1)
			out_state = 2;
		break;

	case 6:
		if (fault == 1)
			out_state = 5;
		else
			out_state = 2;
		break;

	default:
		if (fault == 1)
			out_state = 5;
		break;
	}
	return out_state;
}
