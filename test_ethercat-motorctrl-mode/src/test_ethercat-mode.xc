
/**
 * \file test_ethercat-mode.xc
 * \brief Test illustrates usage of Motor Control with Ethercat
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
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

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <qei_server.h>
#include <hall_server.h>
#include <adc_server_ad7949.h>
#include <velocity_ctrl_server.h>
#include <position_ctrl_server.h>
#include <torque_ctrl_server.h>
#include <ecat_motor_drive.h>
#include <bldc_motor_config.h>
#include <flash_somanet.h>
#include <gpio_server.h>

#define COM_TILE 0
#define IFM_TILE 3

on stdcore[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_TILE]: clock clk_pwm = XS1_CLKBLK_REF;

int main(void)
{
	// Motor control channels
	chan c_adc, c_adctrig;													// adc channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6 ;		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;	// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;	// commutation channels
	chan c_pwm_ctrl;														// pwm channels
	chan c_velocity_ctrl;													// velocity control channel
	chan c_torque_ctrl;														// torque control channel
	chan c_position_ctrl;													// position control channel
	chan c_watchdog; 														// watchdog channel
	chan c_gpio_p1, c_gpio_p2;												// gpio digital channels

	// EtherCat Communication channels
	chan coe_in; 															// CAN from module_ethercat to consumer
	chan coe_out; 															// CAN from consumer to module_ethercat
	chan eoe_in; 															// Ethernet from module_ethercat to consumer
	chan eoe_out; 															// Ethernet from consumer to module_ethercat
	chan eoe_sig;
	chan foe_in; 															// File from module_ethercat to consumer
	chan foe_out; 															// File from consumer to module_ethercat
	chan pdo_in;
	chan pdo_out;
	chan c_sig_1;

	par
	{
		/* Ethercat Communication Handler Loop */
		on stdcore[COM_TILE] :
		{
			ecat_init();
			ecat_handler(coe_out, coe_in, eoe_out, eoe_in, eoe_sig, foe_out,
					foe_in, pdo_out, pdo_in);
		}

		/* Firmware Update Loop */
		on stdcore[COM_TILE] :
		{
			firmware_update_loop(p_spi_flash, foe_out, foe_in, c_sig_1); // firmware update over Ethercat
		}

		/* Ethercat Motor Drive Loop */
		on stdcore[1] :
		{
			ecat_motor_drive(pdo_out, pdo_in, coe_out, c_signal, c_hall_p5, c_qei_p5,\
					c_torque_ctrl, c_velocity_ctrl, c_position_ctrl, c_gpio_p1);
		}

		on stdcore[2]:
		{
			par
			{
				/* Position Control Loop */
				{
					 ctrl_par position_ctrl_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_position_control_param(position_ctrl_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p4,\
							 c_qei_p4, c_position_ctrl, c_commutation_p3);
				}

				/* Velocity Control Loop */
				{
					 ctrl_par velocity_ctrl_params;
					 filter_par sensor_filter_params;
					 hall_par hall_params;
					 qei_par qei_params;

					 init_velocity_control_param(velocity_ctrl_params);
					 init_sensor_filter_param(sensor_filter_params);
					 init_hall_param(hall_params);
					 init_qei_param(qei_params);

					 velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params,\
							 qei_params, SENSOR_USED, c_hall_p3, c_qei_p3, c_velocity_ctrl, c_commutation_p2);
				}

				/* Torque Control Loop */
				{
					ctrl_par torque_ctrl_params;
					hall_par hall_params;
					qei_par qei_params;

					init_qei_param(qei_params);
					init_hall_param(hall_params);
					init_torque_control_param(torque_ctrl_params);

					torque_control( torque_ctrl_params, hall_params, qei_params, SENSOR_USED, c_adc, \
							c_commutation_p1,  c_hall_p2,  c_qei_p2, c_torque_ctrl);
				}

			}
		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on stdcore[IFM_TILE]:
		{
			par
			{
				/* ADC loop */
				adc_ad7949_triggered(c_adc, c_adctrig, clk_adc,
						p_ifm_adc_sclk_conv_mosib_mosia, p_ifm_adc_misoa,
						p_ifm_adc_misob);

				/* PWM Loop */
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					commutation_init_ecat(c_signal, hall_params, qei_params, commutation_params);
					commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, 	\
						c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl,\
						p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2,\
						hall_params, qei_params, commutation_params);						// channel priority 1,2,3
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* GPIO Digital Server */
				gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);

				/* Hall Server */
				{
					hall_par hall_params;
					hall_init_ecat(c_hall_p5, hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, \
							p_ifm_hall, hall_params); 		// channel priority 1,2..6
				}

				/* QEI Server */
				{
					qei_par qei_params;
					qei_init_ecat(c_qei_p5, qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, \
							qei_params);  					// channel priority 1,2..6
				}

			}
		}

	}

	return 0;
}
