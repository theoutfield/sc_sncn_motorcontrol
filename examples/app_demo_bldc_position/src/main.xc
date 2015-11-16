/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_position-ctrl.xc
 * @brief Test illustrates usage of profile position control
 * @author Synapticon GmbH (www.synapticon.com)
 */

#include <print.h>
#include <hall_server.h>
#include <pwm_service_inv.h>
#include <commutation_server.h>
#include <refclk.h>
#include <xscope.h>
#include <qei_server.h>
#include <profile.h>
#include <position_ctrl_server.h>
#include <drive_modes.h>
#include <statemachine.h>
#include <profile_control.h>
#include <drive_modes.h>
#include <position_ctrl_client.h>
#include <internal_config.h>
//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>


on tile[IFM_TILE]: clock clk_adc = XS1_CLKBLK_1;

PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;


/* Test Profile Position function */
void position_profile_test(chanend c_position_ctrl, chanend c_qei, chanend c_hall)
{
	int actual_position = 0;			// ticks
	int target_position = 16000;		// HALL: 4096 extrapolated ticks x nr. pole pairs = one rotation; QEI: your encoder documented resolution x 4 = one rotation
	int velocity 		= 500;			// rpm
	int acceleration 	= 500;			// rpm/s
	int deceleration 	= 500;     	// rpm/s
	int follow_error;
	timer t;
	hall_par hall_params;
	qei_par qei_params;
	init_qei_param(qei_params);
	init_hall_param(hall_params);

	/* Initialise Profile Limits for position profile generator and select position sensor */
	init_position_profile_limits(MAX_ACCELERATION, MAX_PROFILE_VELOCITY, qei_params, hall_params, \
			SENSOR_USED, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT);


	/* Set new target position for profile position control */
	set_profile_position(target_position, velocity, acceleration, deceleration, SENSOR_USED, c_position_ctrl);

	while(1)
	{
	    /* Read actual position from the Position Control Server */
		actual_position = get_position(c_position_ctrl);
		follow_error = target_position - actual_position;

		xscope_int(ACTUAL_POSITION, actual_position);
		xscope_int(TARGET_POSITION, target_position);
		xscope_int(FOLLOW_ERROR, follow_error);

		wait_ms(1, 1, t);  /* 1 ms wait */
	}
}

int main(void)
{
	// Motor control channels
	chan c_qei_p1, c_qei_p2, c_qei_p5;		// qei channels
	chan c_hall_p1, c_hall_p2, c_hall_p5;	// hall channels
	chan c_commutation_p3;	                // commutation channels
	chan c_pwm_ctrl, c_adctrig;				// pwm channels
	chan c_position_ctrl;					// position control channel
	interface WatchdogInterface wd_interface;
	interface CommutationInterface commutation_interface[3];

	par
	{
		/* Test Profile Position Client function*/
		on tile[APP_TILE_1]:
		{
			position_profile_test(c_position_ctrl, c_qei_p5, c_hall_p5);		// test PPM on slave side
			//position_ctrl_unit_test(c_position_ctrl, c_qei_p5, c_hall_p5); 	// Unit test controller
		}


		on tile[APP_TILE_1]:
		{
			/* Position Control Loop */
			{
				 ctrl_par position_ctrl_params;
				 hall_par hall_params;
				 qei_par qei_params;

				 /* Initialize PID parameters for Position Control (defined in config/motor/bldc_motor_config.h) */
				 init_position_control_param(position_ctrl_params);

				 /* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
				 init_hall_param(hall_params);
				 init_qei_param(qei_params);

				 /* Control Loop */
				 position_control(position_ctrl_params, hall_params, qei_params, SENSOR_USED, c_hall_p2,\
						 c_qei_p2, c_position_ctrl, commutation_interface[0]);
			}

		}

		/************************************************************
		 * IFM_TILE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
                do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, pwm_ports);

				/* Motor Commutation loop */
				{
					hall_par hall_params;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_param(hall_params);
					init_qei_param(qei_params);
					commutation_sinusoidal(c_hall_p1,  c_qei_p1, null, wd_interface,
							commutation_interface, c_pwm_ctrl, fet_driver_ports,
							hall_params, qei_params, commutation_params);
				}

				/* Watchdog Server */
                run_watchdog(wd_interface, wd_ports);

				/* Hall Server */
				{
					hall_par hall_params;
#ifdef DC1K
                    //connector 1 is configured as hall
                    p_ifm_encoder_hall_select_ext_d4to5 <: 0b0010;//last two bits define the interface [con2, con1], 0 - hall, 1 - QEI.
#endif
                    run_hall(c_hall_p1, c_hall_p2, null, null, c_hall_p5,null, p_ifm_hall, hall_params); // channel priority 1,2..6
				}

				/* QEI Server */
				{
					qei_par qei_params;

					//connector 2 is configured as QEI
                    run_qei(c_qei_p1, c_qei_p2, null, null, c_qei_p5, null, p_ifm_encoder, qei_params);          // channel priority 1,2..5
				}
			}
		}

	}

	return 0;
}
