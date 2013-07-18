/*
 * drive_ctrl.c
 *
 *  Created on: Jul 17, 2013
 *      Author: pkanajar
 */
#include <drive_ctrl.h>
#include <stdio.h>

int check_ready(int status_word) {
	return (status_word & READY_TO_SWITCH_ON_STATE);
}

int check_switch_enable(int status_word) {
	return (~((status_word & SWITCH_ON_DISABLED_STATE) >> 6) & 0x1);
}

int check_switch_on(int status_word) {
	return (status_word & SWITCHED_ON_STATE) >> 1;
}

int check_op_enable(int status_word) {
	return (status_word & OPERATION_ENABLED_STATE) >> 2;
}

