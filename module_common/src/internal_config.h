
/**
 *
 * \file internal_config.h
 *
 *
 * Copyright (c) 2013, Synapticon GmbH
 * All rights reserved.
 * Author: Pavan Kanajar <pkanajar@synapticon.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Execution of this software or parts of it exclusively takes place on hardware
 *    produced by Synapticon GmbH.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Synapticon GmbH.
 *
 */

#ifndef INTERNAL_CONFIG_H_
#define INTERNAL_CONFIG_H_
#pragma once

// default internal definitions (do not change)

#define TORQUE_CTRL_READ(x)			c_torque_ctrl :> x
#define TORQUE_CTRL_WRITE(x)		c_torque_ctrl <: x
#define TORQUE_CTRL_ENABLE()		c_torque_ctrl <: 1
#define TORQUE_CTRL_DISABLE()		c_torque_ctrl <: 0

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

#define SET    						1
#define UNSET  						0

#define HALL 						1
#define QEI 						2


#define QEI_WITH_INDEX				1
#define QEI_WITH_NO_INDEX 			0

#define DC100_RESOLUTION 			740
#define DC300_RESOLUTION			264

#define INIT_BUSY 					0
#define INIT						1
#define CHECK_BUSY  				10

#define SET_COMM_PARAM_ECAT 		20
#define SET_HALL_PARAM_ECAT 		21
#define SET_QEI_PARAM_ECAT  		22
#define SET_POSITION_CTRL_HALL 		23
#define SET_POSITION_CTRL_QEI		24


#define SET_VELOCITY_CTRL_HALL 		25
#define SET_VELOCITY_CTRL_QEI		26
#define INIT_VELOCITY_CTRL  		29
#define SET_VELOCITY_FILTER 		30
#define FILTER_SIZE 				8                           //default
#define FILTER_SIZE_MAX 			16							//max size
#define SET_VELOCITY_TOKEN 			50
#define GET_VELOCITY_TOKEN 			60
#define SHUTDOWN_VELOCITY 			200
#define ENABLE_VELOCITY				250

#define SET_TORQUE_CTRL_HALL 		27
#define SET_TORQUE_CTRL_QEI  		28
#define SET_TORQUE_TOKEN 			40
#define GET_TORQUE_TOKEN 			41
#define SHUTDOWN_TORQUE	 			201
#define ENABLE_TORQUE				251

#define SET_POSITION_TOKEN 			40
#define GET_POSITION_TOKEN 			41
#define HALL_PRECISION				2
#define QEI_PRECISION				512

#define SHUTDOWN_POSITION 			201
#define ENABLE_POSITION				251

#define SET_VOLTAGE    				2
#define SET_COMMUTATION_PARAMS 		3

#define SET_CTRL_PARAMETER 			100
#define SENSOR_SELECT      			150

#define SUCCESS 					1
#define ERROR   					0

#endif /* INTERNAL_CONFIG_H_ */
