
/**
 * \file hall_config.h
 *
 *	Hall Sensor Config
 *
 * The copyrights, all other intellectual and industrial
 * property rights are retained by XMOS and Synapticon GmbH.
 *
 * Copyright 2013, Synapticon GmbH & XMOS Ltd. All rights reserved.
 * Authors:  Pavan Kanajar <pkanajar@synapticon.com> & Ludwig Orgler <orgler@tin.it>
 *
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code arse still covered by the
 * copyright notice above.
 */

#ifndef __HALL_CONFIG_H__
#define __HALL_CONFIG_H__

#define RPM_CONST 						 60000000  		// 60 s/ 1us
#define HALL_POS_REQ  							1
#define HALL_VELOCITY_REQ 						2
#define HALL_ABSOLUTE_POS_REQ 					3
#define FILTER_LENGTH_HALL 						8

#endif /* __HALL_CONFIG_H__ */
