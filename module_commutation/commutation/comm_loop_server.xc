
/**
 * \file comm_loop_server.xc
 * \brief Commutation Loop based on sinusoidal commutation method
 * \author Ludwig Orgler <lorgler@synapticon.com>
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 * \version 1.0
 * \date 10/04/2014
 */

/*
 * Commutation Loop based on sinusoidal commutation method
 *
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

#include "comm_loop_server.h"
#include <xs1.h>
#include <stdint.h>
#include <pwm_config.h>
#include "pwm_cli_inv.h"
#include "a4935.h"
#include "sine_table_big.h"
#include "adc_client_ad7949.h"
#include "hall_client.h"
#include <xscope.h>
#include "refclk.h"
#include "qei_client.h"
#include <internal_config.h>
#include "print.h"


void commutation_init_to_zero(chanend c_pwm_ctrl, t_pwm_control &pwm_ctrl)
{
	unsigned int pwm[3] = {0, 0, 0};  // PWM OFF
	pwm_share_control_buffer_address_with_server(c_pwm_ctrl, pwm_ctrl);
	update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);
}


int absolute(int var)
{
	if(var < 0)
		var = 0 - var;
	return var;
}



/* Sinusoidal based commutation functions */

void commutation_client_hanlder(chanend c_commutation, int command, commutation_par &commutation_params, \
		int &voltage, int &sensor_select, int init_state, int &shutdown)
{
	switch(command)
	{
		case SET_VOLTAGE:				// set voltage
			c_commutation :> voltage;	//STAR_WINDING
			if(commutation_params.winding_type == DELTA_WINDING)
			{
				voltage = 0 - voltage;
			}
			break;

		case SET_COMMUTATION_PARAMS:
			c_commutation :> commutation_params.angle_variance;
			c_commutation :> commutation_params.max_speed_reached;
			c_commutation :> commutation_params.hall_offset_clk;
			c_commutation :> commutation_params.hall_offset_cclk;
			c_commutation :> commutation_params.winding_type;
			break;

		case SENSOR_SELECT:
			c_commutation :> sensor_select;
			break;

		case CHECK_BUSY:		// init signal
			c_commutation <: init_state;
			break;

		case DISABLE_FETS:
			shutdown = 1;
			break;

		case ENABLE_FETS:
			shutdown = 0;
			voltage = 0;
			break;

		case FETS_STATE:
			c_commutation <: shutdown;
			break;

		default:
			break;
	}
}

void commutation_sinusoidal_loop(port p_ifm_ff1, port p_ifm_ff2, port p_ifm_coastn, int sensor_select, t_pwm_control &pwm_ctrl, hall_par &hall_params, qei_par &qei_params,
		commutation_par &commutation_params, int init_state, chanend c_hall, chanend c_qei,	chanend c_pwm_ctrl,
		chanend c_signal, chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3)
{
	unsigned int command;
	unsigned int pwm[3] = { 0, 0, 0 };
	int angle_pwm = 0;
	int angle = 0;
	int angle_rpm   = 0;
	int speed = 0;
	timer t;
	unsigned int ts;
	int voltage = 0;
	int direction = 0;
	//int init_state = INIT;
	int pwm_half = PWM_MAX_VALUE>>1;
	int max_count_per_hall = qei_params.real_counts/hall_params.pole_pairs;
	int angle_offset = 682/(2*hall_params.pole_pairs);

	int fw_flag = 0;
	int bw_flag = 0;
	int status = 0;
	int nominal_speed;
	int shutdown = 0; //Disable FETS
	int port_a, port_b, check_fet;
	qei_velocity_par qei_velocity_params;
	init_qei_velocity_params(qei_velocity_params);
	//printintln(commutation_params.hall_offset_clk);
	//printintln(commutation_params.hall_offset_cclk);
	//printintln(commutation_params.winding_type);
	//p_ifm_coastn :> check_fet;
	//xscope_probe_data(2, check_fet);
	while (1)
	{
		//p_ifm_coastn :> check_fet;
		//p_ifm_ff1 :> port_a;
		//		p_ifm_ff2 :> port_b;
		//		xscope_probe_data(0, port_a);
		//		xscope_probe_data(1, port_b);
		//		xscope_probe_data(2, check_fet);
		if(sensor_select == HALL) //hall only
		{
			speed = get_hall_velocity(c_hall, hall_params);
			angle = get_hall_position(c_hall);
			angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;
		}
		else if(sensor_select == QEI)
		{
			//angle = (get_sync_position(c_sync) << 12)/max_count_per_hall;
			{angle, fw_flag, bw_flag} = get_qei_sync_position(c_qei);
			angle = (angle << 12)/max_count_per_hall;
			if(voltage >=0)
			{
				if(fw_flag == 0)
				{
					angle = get_hall_position(c_hall);
				}
			}
			else if(voltage <0)
			{
				if(bw_flag == 0)
				{
					angle = get_hall_position(c_hall);
				}
			}
			angle_rpm = (absolute(speed)*commutation_params.angle_variance)/commutation_params.max_speed_reached;
		}
//xscope_probe_data(0, direction);
		if(voltage<0)
			direction = -1;
		else if(voltage >= 0)
			direction = 1;

		if(shutdown == 1)
		{
			pwm[0] = -1;
			pwm[1] = -1;
			pwm[2] = -1;
		}
		else
		{
			if (direction == 1)
			{
				if(sensor_select == HALL)
				{
					angle_pwm = (((angle + angle_rpm + commutation_params.hall_offset_clk - commutation_params.angle_variance) & 0x0fff) >> 2)&0x3ff;//
				}
				else if(sensor_select == QEI)
				{
					angle_pwm = (((angle + commutation_params.qei_forward_offset) & 0x0fff) >> 2)&0x3ff;	 //512
				}
				pwm[0] = ((sine_third_expanded(angle_pwm))*voltage)/CONTROL_LIMIT_PWM   + pwm_half;		// 6944 -- 6867range
				angle_pwm = (angle_pwm + 341) & 0x3ff;

				pwm[1] = ((sine_third_expanded(angle_pwm))*voltage)/CONTROL_LIMIT_PWM   + pwm_half;
				angle_pwm = (angle_pwm + 342) & 0x3ff;
				pwm[2] = ((sine_third_expanded(angle_pwm))*voltage)/CONTROL_LIMIT_PWM   + pwm_half;

			}
			else if (direction == -1)
			{
				if(sensor_select == HALL)
				{
					angle_pwm = (((angle - angle_rpm + commutation_params.hall_offset_cclk + commutation_params.angle_variance) & 0x0fff) >> 2)&0x3ff;
				}
				else if(sensor_select == QEI)
				{
					angle_pwm = (((angle  + commutation_params.qei_backward_offset ) & 0x0fff) >> 2)&0x3ff;  	 //3100
				}
				pwm[0] = ((sine_third_expanded(angle_pwm))*-voltage)/CONTROL_LIMIT_PWM   + pwm_half;
				angle_pwm = (angle_pwm + 341) & 0x3ff;

				pwm[1] = ((sine_third_expanded(angle_pwm))*-voltage)/CONTROL_LIMIT_PWM   + pwm_half;
				angle_pwm = (angle_pwm + 342) & 0x3ff;
				pwm[2] = ((sine_third_expanded(angle_pwm))*-voltage)/CONTROL_LIMIT_PWM   + pwm_half;

			}

			if(pwm[0] < PWM_MIN_LIMIT)
				pwm[0] = 0;
			if(pwm[1] < PWM_MIN_LIMIT)
				pwm[1] = 0;
			if(pwm[2] < PWM_MIN_LIMIT)
				pwm[2] = 0;
		}

   		update_pwm_inv(pwm_ctrl, c_pwm_ctrl, pwm);

		#pragma ordered
		select
		{
			case c_commutation_p1 :> command:
				commutation_client_hanlder( c_commutation_p1, command, commutation_params, voltage, \
						sensor_select, init_state, shutdown);
				break;

			case c_commutation_p2 :> command:
				commutation_client_hanlder( c_commutation_p2, command, commutation_params, voltage,
						sensor_select, init_state, shutdown);
				break;

			case c_commutation_p3 :> command:
				commutation_client_hanlder( c_commutation_p3, command, commutation_params, voltage,
						sensor_select, init_state, shutdown);
				break;

			case c_signal :> command:
				if(command == CHECK_BUSY)			// init signal
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
					//pwm[0] = 0;
					//pwm[1] = 0;
					//pwm[2] = 0;
					// angle_pwm = 0;
					// angle = 0;
					// angle_rpm   = 0;
					// speed = 0;
					voltage = 0;
					//direction = 0;
					max_count_per_hall = qei_params.real_counts/hall_params.pole_pairs;
					angle_offset = 682/(2*hall_params.pole_pairs);
					fw_flag = 0;
					bw_flag = 0;
					//init_qei_velocity_params(qei_velocity_params);
				}
				break;

			default:
				break;
		}

	}

}

void commutation_sinusoidal(chanend c_hall, chanend c_qei, chanend c_signal, chanend c_watchdog, \
		chanend  c_commutation_p1, chanend  c_commutation_p2, chanend  c_commutation_p3, chanend c_pwm_ctrl,\
		out port p_ifm_esf_rstn_pwml_pwmh, port p_ifm_coastn, port p_ifm_ff1, port p_ifm_ff2,\
		hall_par &hall_params, qei_par &qei_params, commutation_par &commutation_params)
{
		const unsigned t_delay = 300*USEC_FAST;
	//const unsigned timeout = 2*SEC_FAST;
		timer t;
		unsigned int ts;
		t_pwm_control pwm_ctrl;
		int check_fet;
		int init_state = INIT_BUSY;
		int port_a, port_b;

		commutation_init_to_zero(c_pwm_ctrl, pwm_ctrl);

		// enable watchdog
		t :> ts;
		t when timerafter (ts + 250000*4):> ts;
		c_watchdog <: WD_CMD_START;

		t :> ts;
		t when timerafter (ts + t_delay) :> ts;

		a4935_init(A4935_BIT_PWML | A4935_BIT_PWMH, p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn);
		t when timerafter (ts + t_delay) :> ts;

		p_ifm_coastn :> check_fet;
		init_state = check_fet;
						//if(check_fet == 1)
							//printstrln("fet enabled");
		//while(1)
		//{
		//	p_ifm_ff1 :> port_a;
		//	p_ifm_ff2 :> port_b;
		//
		//	xscope_probe_data(0, port_a);
		//	xscope_probe_data(1, port_b);
		//}


		commutation_sinusoidal_loop(p_ifm_ff1, p_ifm_ff2, p_ifm_coastn, HALL, pwm_ctrl, hall_params, qei_params, commutation_params, init_state,\
				  c_hall, c_qei, c_pwm_ctrl, c_signal, c_commutation_p1, c_commutation_p2, \
				  c_commutation_p3);
}
