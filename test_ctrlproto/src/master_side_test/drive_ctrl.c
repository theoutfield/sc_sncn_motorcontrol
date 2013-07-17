/*
 * drive_ctrl.c
 *
 *  Created on: Jul 17, 2013
 *      Author: pkanajar
 */
#include <drive_ctrl.h>

int check_ready(int status_word) {
	//int ready =
	return (status_word & READY_TO_SWITCH_ON_STATE);
}
int check_switch_enable(int status_word) {
	//int switch_enable =
	return (status_word & SWITCH_ON_DISABLED_STATE) >> 6;
}
int check_switch_on(int status_word) {
	//int switch_on =
	return (status_word & SWITCHED_ON_STATE) >> 1;
}
int check_op_enable(int status_word) {
	//int op_enable =
	return (status_word & OPERATION_ENABLED_STATE) >> 2;
}
void run_drive() {
	int ready = 0;
	int switch_enable = 0;
	int status_word;
	int switch_on = 0;
	int op_enable = 0;

	/* setup */
	//status_word = read_status();

	ready = check_ready(status_word);

	switch_enable = check_switch_enable(status_word);

	switch_on = check_switch_on(status_word);

	op_enable = check_op_enable(status_word);

}
