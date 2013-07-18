/*
 * drive_ctrl.h
 *
 *  Created on: Jul 17, 2013
 *      Author: pkanajar
 */

#ifndef DRIVE_CTRL_H_
#define DRIVE_CTRL_H_
#pragma once
#include <drive_config.h>

extern int check_ready(int status_word);

extern int check_switch_enable(int status_word);

extern int check_switch_on(int status_word);

extern int check_op_enable(int status_word);

#endif /* DRIVE_CTRL_H_ */
