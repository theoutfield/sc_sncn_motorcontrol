
/**
 * \file hall_config.h
 * \brief Hall Sensor Config Definitions
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

 

#ifndef __HALL_CONFIG_H__
#define __HALL_CONFIG_H__

#define RPM_CONST 						 60000000  		// 60 s/ 1us
#define HALL_POS_REQ  							1
#define HALL_VELOCITY_REQ 						2
#define HALL_ABSOLUTE_POS_REQ 					3
#define FILTER_LENGTH_HALL 						16
#define RESET_HALL_COUNT						9

#endif /* __HALL_CONFIG_H__ */
