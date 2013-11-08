#ifndef INTERNAL_CONFIG_H_
#define INTERNAL_CONFIG_H_
#pragma once

#define VELOCITY_CTRL_ENABLE()   	c_velocity_ctrl <: 1
#define VELOCITY_CTRL_DISABLE()  	c_velocity_ctrl <: 0
#define VELOCITY_CTRL_READ(x)		c_velocity_ctrl :> x
#define VELOCITY_CTRL_WRITE(x)		c_velocity_ctrl <: x

#define POSITION_CTRL_ENABLE()   	c_position_ctrl <: 1
#define POSITION_CTRL_DISABLE()  	c_position_ctrl <: 0
#define POSITION_CTRL_READ(x)		c_position_ctrl :> x
#define POSITION_CTRL_WRITE(x)		c_position_ctrl <: x

#define SIGNAL_READ(x) 				c_signal :> x
#define SIGNAL_WRITE(x)				c_signal <: x

#define SET    	1
#define UNSET  	0

#define HALL 		1
#define QEI 		2
#define INIT_BUSY 	0
#define INIT		1
#define CHECK_BUSY  10

#define SET_COMM_PARAM_ECAT 20
#define SET_HALL_PARAM_ECAT 21
#define SET_QEI_PARAM_ECAT  22
#define SET_POS_CTRL_HALL 	23
#define SET_POS_CTRL_QEI	24
#define SET_VEL_CTRL_HALL 	25
#define SET_VEL_CTRL_QEI	26
#define INIT_VELOCITY_CTRL  29
#define SET_VELOCITY_FILTER 30
#define SET_TORQUE_CTRL_HALL 27
#define SET_TORQUE_CTRL_QEI  28

#define SUCCESS 1
#define ERROR   0 //based on timeout for success

#endif /* INTERNAL_CONFIG_H_ */
