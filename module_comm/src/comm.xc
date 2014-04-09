
/**
 * \file comm.xc
 * \brief Ctrlproto data struct client
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

/*
 * Copyright (c) 2014, Synapticon GmbH
 * All rights reserved.
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

#include <comm.h>


int get_target_torque(ctrl_proto_values_t InOut)
{
	return InOut.target_torque;
}

int get_target_velocity(ctrl_proto_values_t InOut)
{
	return InOut.target_velocity;
}

int get_target_position(ctrl_proto_values_t InOut)
{
	return InOut.target_position;
}

void send_actual_torque(int actual_torque, ctrl_proto_values_t &InOut)
{
	InOut.torque_actual = actual_torque;
}

void send_actual_velocity(int actual_velocity, ctrl_proto_values_t &InOut)
{
	InOut.velocity_actual = actual_velocity;
}

void send_actual_position(int actual_position, ctrl_proto_values_t &InOut)
{
	InOut.position_actual = actual_position;
}

void update_hall_param_ecat(hall_par &hall_params, chanend coe_out)
{
	int min;
	int max;
	{hall_params.pole_pairs, max, min} = hall_sdo_update(coe_out);
	if(max >= 0 && min >= 0)
	{
		if(max > min)
			hall_params.max_ticks = max;
		else
			hall_params.max_ticks = min;
	}
	else if(max <= 0 && min <= 0)
	{
		if(max < min)
			hall_params.max_ticks = 0 - max;
		else
			hall_params.max_ticks = 0 - min;
	}
	else if(max > 0 && min < 0)
	{
		if(max > 0 - min)
			hall_params.max_ticks = max;
		else
			hall_params.max_ticks = 0 - min;
	}
	else if(max < 0 && min > 0)
	{
		if(min > 0 - max)
			hall_params.max_ticks = min;
		else
			hall_params.max_ticks = 0 - max;
	}
	hall_params.max_ticks_per_turn = hall_params.pole_pairs * 4096;
	hall_params.max_ticks += hall_params.max_ticks_per_turn;
}

void update_qei_param_ecat(qei_par &qei_params, chanend coe_out)
{
	int min;
	int max;
	int sensor_polarity;
	{qei_params.real_counts, max, min, qei_params.index, sensor_polarity} = qei_sdo_update(coe_out);

	if(max >= 0 && min >= 0)
	{
		if(max > min)
			qei_params.max_ticks = max;
		else
			qei_params.max_ticks = min;
	}
	else if(max <= 0 && min <= 0)
	{
		if(max < min)
			qei_params.max_ticks = 0 - max;
		else
			qei_params.max_ticks = 0 - min;
	}
	else if(max > 0 && min < 0)
	{
		if(max > 0 - min)
			qei_params.max_ticks = max;
		else
			qei_params.max_ticks = 0 - min;
	}
	else if(max < 0 && min > 0)
	{
		if(min > 0 - max)
			qei_params.max_ticks = min;
		else
			qei_params.max_ticks = 0 - max;
	}

	qei_params.max_ticks_per_turn = __qei_max_counts(qei_params.real_counts);
	qei_params.max_ticks += qei_params.max_ticks_per_turn;  // tolerance

	//printstr(" value ");
	//printintln(qei_params.max_ticks_per_turn );
	//printintln(qei_params.max_ticks);
	sensor_polarity = sensor_polarity&0x01;

	switch(sensor_polarity)
	{
		case 0:
			//printstr(" nor ");
			sensor_polarity = NORMAL;
			break;

		case 1:
			//printstr(" inv ");
			sensor_polarity = INVERTED;
			break;

		default:
			break;
	}
	//printintln(sensor_polarity);
	qei_params.sensor_polarity = sensor_polarity;
}

void update_commutation_param_ecat(commutation_par &commutation_params, chanend coe_out)
{
	{commutation_params.hall_offset_clk, commutation_params.hall_offset_cclk, \
		commutation_params.winding_type} = commutation_sdo_update(coe_out);
}

void update_cst_param_ecat(cst_par &cst_params, chanend coe_out)
{
	{cst_params.nominal_current, cst_params.nominal_motor_speed, cst_params.polarity,\
		cst_params.max_torque, cst_params.motor_torque_constant} = cst_sdo_update(coe_out);
	if(cst_params.polarity >= 0)
		cst_params.polarity = 1;
	else if(cst_params.polarity < 0)
		cst_params.polarity = -1;
}

void update_csv_param_ecat(csv_par &csv_params, chanend coe_out)
{
	{csv_params.max_motor_speed, csv_params.nominal_current, csv_params.polarity, \
		csv_params.max_acceleration, csv_params.motor_torque_constant} = csv_sdo_update(coe_out);
		//return {max_motor_speed, nominal_current, polarity, max_acceleration, motor_torque_constant};
	if(csv_params.polarity >= 0)
		csv_params.polarity = 1;
	else if(csv_params.polarity < 0)
		csv_params.polarity = -1;
}

void update_csp_param_ecat(csp_par &csp_params, chanend coe_out)
{
	{csp_params.base.max_motor_speed, csp_params.base.polarity, csp_params.base.nominal_current, \
		csp_params.min_position_limit, csp_params.max_position_limit, \
		csp_params.base.max_acceleration} = csp_sdo_update(coe_out);
	if(csp_params.base.polarity >= 0)
		csp_params.base.polarity = 1;
	else if(csp_params.base.polarity < 0)
		csp_params.base.polarity = -1;
}

void update_pt_param_ecat(pt_par &pt_params, chanend coe_out)
{
	{pt_params.profile_slope, pt_params.polarity} = pt_sdo_update(coe_out);
	if(pt_params.polarity >= 0)
		pt_params.polarity = 1;
	else if(pt_params.polarity < 0)
		pt_params.polarity = -1;
}

void update_pv_param_ecat(pv_par &pv_params, chanend coe_out)
{
	{pv_params.max_profile_velocity, pv_params.profile_acceleration, \
		pv_params.profile_deceleration,\
	 	pv_params.quick_stop_deceleration,\
		pv_params.polarity} = pv_sdo_update(coe_out);
}

void update_pp_param_ecat(pp_par &pp_params, chanend coe_out)
{
	{pp_params.base.max_profile_velocity, pp_params.profile_velocity, \
		pp_params.base.profile_acceleration, pp_params.base.profile_deceleration,\
	 	pp_params.base.quick_stop_deceleration,\
		pp_params.software_position_limit_min, \
		pp_params.software_position_limit_max, \
		pp_params.base.polarity,\
		pp_params.max_acceleration} = pp_sdo_update(coe_out);
}

void update_torque_ctrl_param_ecat(ctrl_par &torque_ctrl_params, chanend coe_out)
{
	{torque_ctrl_params.Kp_n, torque_ctrl_params.Ki_n, torque_ctrl_params.Kd_n} = torque_sdo_update(coe_out);
	torque_ctrl_params.Kp_d = 65536; 				// 16 bit precision PID gains
	torque_ctrl_params.Ki_d = 65536;
	torque_ctrl_params.Kd_d = 65536;

	torque_ctrl_params.Loop_time = 1 * MSEC_STD;  	// units - core timer value //CORE 2/1/0 default

	torque_ctrl_params.Control_limit = CONTROL_LIMIT_PWM - 150;  // PWM resolution

	if(torque_ctrl_params.Ki_n != 0)    			// auto calculated using control_limit
		torque_ctrl_params.Integral_limit = torque_ctrl_params.Control_limit * (torque_ctrl_params.Ki_d/torque_ctrl_params.Ki_n) ;
	else
		torque_ctrl_params.Integral_limit = 0;
	return;
}


void update_velocity_ctrl_param_ecat(ctrl_par &velocity_ctrl_params, chanend coe_out)
{
	{velocity_ctrl_params.Kp_n, velocity_ctrl_params.Ki_n, velocity_ctrl_params.Kd_n} = velocity_sdo_update(coe_out);
	velocity_ctrl_params.Kp_d = 65536; 				// 16 bit precision PID gains
	velocity_ctrl_params.Ki_d = 65536;
	velocity_ctrl_params.Kd_d = 65536;

	velocity_ctrl_params.Loop_time = 1 * MSEC_STD;  // units - core timer value //CORE 2/1/0 default

	velocity_ctrl_params.Control_limit = CONTROL_LIMIT_PWM - 150; // PWM resolution

	if(velocity_ctrl_params.Ki_n != 0)    			// auto calculated using control_limit
		velocity_ctrl_params.Integral_limit = velocity_ctrl_params.Control_limit * (velocity_ctrl_params.Ki_d/velocity_ctrl_params.Ki_n) ;
	else
		velocity_ctrl_params.Integral_limit = 0;
	return;
}

void update_position_ctrl_param_ecat(ctrl_par &position_ctrl_params, chanend coe_out)
{
	{position_ctrl_params.Kp_n, position_ctrl_params.Ki_n, position_ctrl_params.Kd_n} = position_sdo_update(coe_out);
	position_ctrl_params.Kp_d = 65536; 				// 16 bit precision PID gains
	position_ctrl_params.Ki_d = 65536;
	position_ctrl_params.Kd_d = 65536;

	position_ctrl_params.Loop_time = 1 * MSEC_STD;  // units - core timer value //CORE 2/1/0 default

	position_ctrl_params.Control_limit = CONTROL_LIMIT_PWM - 150; // PWM resolution

	if(position_ctrl_params.Ki_n != 0)    			// auto calculated using control_limit
		position_ctrl_params.Integral_limit = position_ctrl_params.Control_limit * (position_ctrl_params.Ki_d/position_ctrl_params.Ki_n) ;
	else
		position_ctrl_params.Integral_limit = 0;
	return;
}


void set_commutation_param_ecat(chanend c_signal, hall_par &hall_params, qei_par &qei_params, \
		commutation_par &commutation_params, int nominal_speed)
{
	c_signal <: SET_COMM_PARAM_ECAT;
	c_signal <: hall_params.pole_pairs;
	c_signal <: qei_params.index;
	c_signal <: qei_params.max_ticks_per_turn;
	c_signal <: qei_params.real_counts;
	c_signal <: nominal_speed;
	c_signal <: commutation_params.hall_offset_clk;
	c_signal <: commutation_params.hall_offset_cclk;
	c_signal <: commutation_params.winding_type;
}

void commutation_init_ecat(chanend c_signal, hall_par &hall_params, qei_par &qei_params, commutation_par &commutation_params)
{
	int command;
	int nominal_speed;
	int flag = 0;
	int init_state = INIT_BUSY;
	while(1)
	{
		#pragma ordered
		select
		{
			case c_signal :> command:
				//printintln(command);
				if(command == CHECK_BUSY)
				{
					c_signal <: init_state;
				}
				else if(command == SET_COMM_PARAM_ECAT)
				{
					c_signal :> hall_params.pole_pairs;
					c_signal :> qei_params.index;
					c_signal :> qei_params.max_ticks_per_turn;
					c_signal :> qei_params.real_counts;
					c_signal :> nominal_speed;
					c_signal :> commutation_params.hall_offset_clk;
					c_signal :> commutation_params.hall_offset_cclk;
					c_signal :> commutation_params.winding_type;
					flag = 1;

					commutation_params.angle_variance = 1024/(hall_params.pole_pairs * 3);
					if(hall_params.pole_pairs < 4)
					{
						commutation_params.max_speed_reached = nominal_speed*4;
						commutation_params.flag = 1;
					}
					else if(hall_params.pole_pairs >=4)
					{
						commutation_params.max_speed_reached = nominal_speed;
						commutation_params.flag = 0;
					}
					commutation_params.qei_forward_offset = 0;
					commutation_params.qei_backward_offset = 0;
					//printstrln("ecat");
//					printintln(hall_params.gear_ratio);
//					printintln(hall_params.pole_pairs);
//					printintln(commutation_params.max_speed_reached );
//										printintln(commutation_params.angle_variance);

				}
				break;

			default:
				break;

		}
		if(flag == 1)
			break;
	}
}


void set_hall_param_ecat(chanend c_hall, hall_par &hall_params)
{
	c_hall <: SET_HALL_PARAM_ECAT;
	c_hall <: hall_params.pole_pairs;
	c_hall <: hall_params.max_ticks;
	c_hall <: hall_params.max_ticks_per_turn;
	//hall_params.

}

void hall_init_ecat(chanend c_hall, hall_par &hall_params)
{
	int command;
	int init_state = INIT_BUSY;
	int flag = 0;
	while(1)
	{
		#pragma ordered
		select
		{
			case c_hall :> command:
				if(command == CHECK_BUSY)
				{
					c_hall <: init_state;
				}
				else if(command == SET_HALL_PARAM_ECAT)
				{
					c_hall :> hall_params.pole_pairs;
					c_hall :> hall_params.max_ticks;
					c_hall :> hall_params.max_ticks_per_turn;
					flag = 1;

					//printintln(hall_params.max_ticks);
					//printintln(hall_params.pole_pairs);

				}
				break;

			default:
				break;

		}
		if(flag == 1)
			break;
	}
}

void set_qei_param_ecat(chanend c_qei, qei_par &qei_params)
{
	c_qei <: SET_QEI_PARAM_ECAT;
	c_qei <: qei_params.index;
	c_qei <: qei_params.max_ticks_per_turn;
	c_qei <: qei_params.real_counts;
	c_qei <: qei_params.poles;
	c_qei <: qei_params.max_ticks;
	c_qei <: qei_params.sensor_polarity;
}

void qei_init_ecat(chanend c_qei, qei_par &qei_params)
{
	int command;
	int init_state = INIT_BUSY;
	int flag = 0;
	while(1)
	{
		#pragma ordered
		select
		{
			case c_qei :> command:
				if(command == CHECK_BUSY)
				{
					c_qei <: init_state;
				}
				else if(command == SET_QEI_PARAM_ECAT)
				{
					c_qei :> qei_params.index;
					c_qei :> qei_params.max_ticks_per_turn;
					c_qei :> qei_params.real_counts;
					c_qei :> qei_params.poles;
					c_qei :> qei_params.max_ticks;
					c_qei :> qei_params.sensor_polarity;
					flag = 1;

//					printintln(qei_params.gear_ratio);
//					printintln(qei_params.index);
//					printintln(qei_params.max_ticks_per_turn);
//					printintln(qei_params.real_counts);
				}
				break;

			default:
				break;

		}
		if(flag == 1)
			break;
	}
}

