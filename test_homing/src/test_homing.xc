
/**
 * \file test_homing-ctrl.xc
 * \brief Test illustrates implementation of homing method with a positive limit switch and a home switch
 * \author Pavan Kanajar <pkanajar@synapticon.com>
 * \author Martin Schwarz <mschwarz@synapticon.com>
 */

#include <xs1.h>
#include <platform.h>
#include <print.h>
#include <ioports.h>
#include <hall_server.h>
#include <qei_server.h>
#include <pwm_service_inv.h>
#include <comm_loop_server.h>
#include <refclk.h>
#include <velocity_ctrl_server.h>
#include <profile.h>
#include <internal_config.h>
#include <bldc_motor_config.h>
#include <drive_config.h>
#include <profile_control.h>
#include "velocity_ctrl_client.h"
#include <qei_client.h>
#include <gpio_server.h>
#include <gpio_client.h>

#define TILE_ONE 0
#define IFM_TILE 3

on stdcore[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;
on stdcore[IFM_TILE]: clock clk_pwm = XS1_CLKBLK_REF;

/* Test homing function */
void homing(chanend c_qei, chanend c_gpio, chanend c_velocity_ctrl)
{
	int homing_velocity = 1000;				// rpm
	int homing_acceleration = 2000;			// rpm/s
	int reset_counter = 0;
	int velocity_ramp;
	int actual_velocity;
	int direction;
	int i = 1;
	timer t;
	unsigned int time;
	int steps;
	int home_state;
	int safety_state;
	int capture_position = 0;
	int current_position = 0;
	int offset = 0;
	int end_state = 0;
	int init_state;
	init_state = __check_velocity_init(c_velocity_ctrl);

	/**< Configure gpio digital port 0 as Switch Input and active high (Home Switch) */
	config_gpio_digital_input(c_gpio, 0, SWITCH_INPUT_TYPE, ACTIVE_HIGH);

	/**< Configure gpio digital port 1 as Switch Input and active high (Safety Limit Switch) */
	config_gpio_digital_input(c_gpio, 1, SWITCH_INPUT_TYPE, ACTIVE_HIGH);

	/**< End configuration of digital ports */
	end_config_gpio(c_gpio);

	while(1)
	{
		init_state = init_velocity_control(c_velocity_ctrl);
		if(init_state == INIT)
			break;
	}

	i = 1;
	actual_velocity = get_velocity(c_velocity_ctrl);
	steps = init_velocity_profile(homing_velocity * HOMING_METHOD, actual_velocity, homing_acceleration, homing_acceleration, MAX_PROFILE_VELOCITY);
	t :> time;
	while(1)
	{
		select
		{

			case t when timerafter(time + MSEC_STD) :> time:
				if(i < steps)
				{
					velocity_ramp = velocity_profile_generate(i);
					i = i+1;
				}
				set_velocity(velocity_ramp, c_velocity_ctrl);
				home_state = read_gpio_digital_input(c_gpio, 0);
				safety_state = read_gpio_digital_input(c_gpio, 1);
				{capture_position, direction} = get_qei_position_absolute(c_qei);

				if(home_state == 1 || safety_state == 1)
				{
					actual_velocity = get_velocity(c_velocity_ctrl);
					steps = init_velocity_profile(0, actual_velocity, homing_acceleration, homing_acceleration, MAX_PROFILE_VELOCITY);
					for(i = 1; i < steps; i++)
					{
						velocity_ramp = velocity_profile_generate(i);
						set_velocity(velocity_ramp, c_velocity_ctrl);
						actual_velocity = get_velocity(c_velocity_ctrl);

						t when timerafter(time + MSEC_STD) :> time;
					}
					end_state = 1;
					shutdown_velocity_ctrl(c_velocity_ctrl);

					if(home_state == 1)
					{
						{current_position, direction} = get_qei_position_absolute(c_qei);
						offset = current_position - capture_position;
						printstr("Homing offset ");
						printintln(offset);
						reset_qei_count(c_qei, offset);
						reset_counter = 1;
					}

				}
				break;
		}
		if(end_state == 1)
			break;
	}

	if(reset_counter == 1)
		printstrln("homing_success");
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6;					// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;				// hall channels
	chan c_commutation_p1, c_commutation_p2, c_commutation_p3, c_signal;				// commutation channels
	chan c_pwm_ctrl, c_adctrig;															// pwm channels
	chan c_velocity_ctrl;																// velocity control channel
	chan c_watchdog; 																	// watchdog channel
	chan c_gpio_p1, c_gpio_p2; 															// gpio Communication channels

	par
	{
		/* Test homing function */
		on stdcore[TILE_ONE]:
		{
			homing(c_qei_p3, c_gpio_p1, c_velocity_ctrl);
		}

		on stdcore[TILE_ONE]:
		{

			/*Velocity Control Loop*/
			{
				ctrl_par velocity_ctrl_params;
				filter_par sensor_filter_params;
				hall_par hall_params;
				qei_par qei_params;

				/* Initialize PID parameters for Velocity Control (defined in config/motor/bldc_motor_config.h) */
				init_velocity_control_param(velocity_ctrl_params);

				/* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
				init_hall_param(hall_params);
				init_qei_param(qei_params);

				/* Initialize sensor filter length */
				init_sensor_filter_param(sensor_filter_params);

				/* Control Loop */
				velocity_control(velocity_ctrl_params, sensor_filter_params, hall_params, \
					 qei_params, SENSOR_USED, c_hall_p2, c_qei_p2, c_velocity_ctrl, c_commutation_p2);
			}

		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on stdcore[IFM_TILE]:
		{
			par
			{
				/* GPIO Digital Server */
				gpio_digital_server(p_ifm_ext_d, c_gpio_p1, c_gpio_p2);

				/* PWM Loop */
				do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, p_ifm_dummy_port,\
						p_ifm_motor_hi, p_ifm_motor_lo, clk_pwm);

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					init_commutation_param(commutation_params, hall_params, MAX_NOMINAL_SPEED); 						// initialize commutation params
					commutation_sinusoidal(c_hall_p1,  c_qei_p1, c_signal, c_watchdog, 	\
								c_commutation_p1, c_commutation_p2, c_commutation_p3, c_pwm_ctrl,\
								p_ifm_esf_rstn_pwml_pwmh, p_ifm_coastn, p_ifm_ff1, p_ifm_ff2,\
								hall_params, qei_params, commutation_params);											// channel priority 1,2,3
				}

				/* Watchdog Server */
				run_watchdog(c_watchdog, p_ifm_wd_tick, p_ifm_shared_leds_wden);

				/* Hall Server */
				{
					hall_par hall_params;
					init_hall_param(hall_params);
					run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6, p_ifm_hall, hall_params); // channel priority 1,2..6
				}

				/* QEI Server */
				{
					qei_par qei_params;
					init_qei_param(qei_params);
					run_qei(c_qei_p1, c_qei_p2, c_qei_p3, c_qei_p4, c_qei_p5, c_qei_p6, p_ifm_encoder, qei_params); 	 // channel priority 1,2..6
				}
			}
		}

	}

	return 0;
}
