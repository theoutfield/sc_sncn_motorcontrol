/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_position-ctrl.xc
 * @brief Test illustrates usage of profile position control
 * @author Synapticon GmbH (www.synapticon.com)
 */

#include <print.h>
#include <refclk.h>

#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service_inv.h>
#include <commutation_service.h>

#include <position_ctrl_service.h>
#include <profile.h>
#include <profile_control.h>

#include <xscope.h>

//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control, interface QEIInterface client i_qei)
{
	int actual_position = 0;			// ticks
	int target_position = 16000;		// HALL: 4096 extrapolated ticks x nr. pole pairs = one rotation; QEI: your encoder documented resolution x 4 = one rotation
	int velocity 		= 500;			// rpm
	int acceleration 	= 500;			// rpm/s
	int deceleration 	= 500;     	// rpm/s
	int follow_error;
	timer t;
	HallConfig hall_config;
	qei_par qei_params;
	init_qei_param(qei_params);
	init_hall_config(hall_config);

	/* Initialise Profile Limits for position profile generator and select position sensor */
	init_position_profile_limits(MAX_ACCELERATION, MAX_PROFILE_VELOCITY, qei_params, hall_config, \
			SENSOR_USED, MAX_POSITION_LIMIT, MIN_POSITION_LIMIT);


	/* Set new target position for profile position control */
	set_profile_position(target_position, velocity, acceleration, deceleration, SENSOR_USED, i_position_control);

	while(1)
	{
	    /* Read actual position from the Position Control Server */
		actual_position = i_position_control.get_position();
		follow_error = target_position - actual_position;

		xscope_int(ACTUAL_POSITION, actual_position);
		xscope_int(TARGET_POSITION, target_position);
		xscope_int(FOLLOW_ERROR, follow_error);

		wait_ms(1, 1, t);  /* 1 ms wait */
	}
}

PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;
HallPorts hall_ports = HALL_PORTS;
EncoderPorts encoder_ports = ENCODER_PORTS;

int main(void)
{
	// Motor control channels
	chan c_pwm_ctrl, c_adctrig;				// pwm channels

	interface WatchdogInterface wd_interface;
	interface CommutationInterface commutation_interface[3];
	interface HallInterface i_hall[5];
	interface QEIInterface i_qei[5];

	interface PositionControlInterface i_position_control;

	par
	{
		/* Test Profile Position Client function*/
		on tile[APP_TILE_1]: position_profile_test(i_position_control, i_qei[2]);      // test PPM on slave side

		on tile[APP_TILE_1]:
		{
			/* Position Control Loop */
			{
				 ctrl_par position_ctrl_params;
				 HallConfig hall_config;
				 qei_par qei_params;

				 /* Initialize PID parameters for Position Control (defined in config/motor/bldc_motor_config.h) */
				 init_position_control_param(position_ctrl_params);

				 /* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
				 init_hall_config(hall_config);
				 init_qei_param(qei_params);

				 /* Control Loop */
				 position_control_service(position_ctrl_params, hall_config, qei_params, SENSOR_USED, i_hall[1],
				         i_qei[1], i_position_control, commutation_interface[0]);
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

                /* Watchdog Server */
                watchdog_service(wd_interface, wd_ports);

                /* QEI Server */
                {
                    qei_velocity_par qei_velocity_params;
                    qei_par qei_config;
                    init_qei_velocity_params(qei_velocity_params);

                    qei_service(i_qei, encoder_ports, qei_config, qei_velocity_params);         // channel priority 1,2..6
                }

                {
                	HallConfig hall_config;
                	 // NEEDS INITIALIZATION
                	hall_service(i_hall, hall_ports, hall_config);
            	}

				/* Motor Commutation loop */
				{
					HallConfig hall_config;
					qei_par qei_params;
					commutation_par commutation_params;
					init_hall_config(hall_config);
					init_qei_param(qei_params);

					commutation_service(i_hall[0], i_qei[0], null, wd_interface, commutation_interface,
					        c_pwm_ctrl, fet_driver_ports, hall_config, qei_params, commutation_params);
				}
            }
        }
    }

	return 0;
}
