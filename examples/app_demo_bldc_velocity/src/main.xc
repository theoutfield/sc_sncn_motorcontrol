/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_velocity-ctrl.xc
 * @brief Test illustrates usage of profile velocity control
 * @author Synapticon GmbH (www.synapticon.com)
 */

#include <print.h>
#include <refclk.h>

#include <pwm_service_inv.h>
#include <hall_service.h>
#include <qei_service.h>
#include <commutation_service.h>

#include <velocity_ctrl_service.h>
#include <profile.h>
#include <profile_control.h>

#include <xscope.h>
//Configure your motor parameters in config/bldc_motor_config.h
#include <bldc_motor_config.h>


PwmPorts pwm_ports = PWM_PORTS;
WatchdogPorts wd_ports = WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = FET_DRIVER_PORTS;
HallPorts hall_ports = HALL_PORTS;
EncoderPorts encoder_ports = ENCODER_PORTS;

#ifdef DC1K
port p_ifm_encoder_hall_select_ext_d4to5 = SELECTION_HALL_ENCODER_PORT;
#endif

/* Test Profile Velocity function */
void profile_velocity_test(interface VelocityControlInterface client i_velocity_control)
{
	int target_velocity = 900;	 		// rpm
	int acceleration 	= 1000;			// rpm/s
	int deceleration 	= 1000;			// rpm/s
	int actual_velocity;
	xscope_int(TARGET_VELOCITY, target_velocity);

	set_profile_velocity( target_velocity, acceleration, deceleration, MAX_PROFILE_VELOCITY, i_velocity_control);

	while(1) {
	    actual_velocity = i_velocity_control.get_velocity();

	    xscope_int(TARGET_VELOCITY, target_velocity);
	    xscope_int(ACTUAL_VELOCITY, actual_velocity);

	    delay_microseconds(1);
	}
}

int main(void)
{
	// Motor control channels
	chan c_pwm_ctrl, c_adctrig;							// pwm channels

	interface WatchdogInterface wd_interface;
    interface CommutationInterface commutation_interface[3];
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];

    interface VelocityControlInterface i_velocity_control;

	par
	{

		/* Test Profile Velocity function */
		on tile[APP_TILE_1]: profile_velocity_test(i_velocity_control);            // test PVM on node

		on tile[APP_TILE_1]:
		{

			/* Velocity Control Loop */
			{
				ctrl_par velocity_ctrl_params;
				filter_par sensor_filter_params;
				HallConfig hall_config;
				qei_par qei_params;

				/* Initialize PID parameters for Velocity Control (defined in config/motor/bldc_motor_config.h) */
				init_velocity_control_param(velocity_ctrl_params);

				/* Initialize Sensor configuration parameters (defined in config/motor/bldc_motor_config.h) */
				init_hall_config(hall_config);
				init_qei_param(qei_params);

				/* Initialize sensor filter length */
				init_sensor_filter_param(sensor_filter_params);

				/* Control Loop */
<<<<<<< HEAD
				velocity_control_service(velocity_ctrl_params, sensor_filter_params, hall_params,
					 qei_params, SENSOR_USED, i_hall[1], i_qei[1], i_velocity_control, commutation_interface[0]);
=======
				velocity_control(velocity_ctrl_params, sensor_filter_params, hall_config,
					 qei_params, SENSOR_USED, i_hall[1], c_qei_p2, c_velocity_ctrl, commutation_interface[0]);
>>>>>>> Resolve conflicts ocured cause Agus removed the hall_par argument from run_hall and other functions
			}

		}

		/************************************************************
		 * IFM_CORE
		 ************************************************************/
		on tile[IFM_TILE]:
		{
			par
			{
				/* PWM Loop */
			    do_pwm_inv_triggered(c_pwm_ctrl, c_adctrig, pwm_ports);

                /* Watchdog Server */
			    watchdog_service(wd_interface, wd_ports);

                /* Hall Server */
<<<<<<< HEAD
                hall_service(i_hall, hall_ports); // channel priority 1,2..5
=======
                {
                	HallConfig hall_config;
                	run_hall(i_hall, hall_ports, hall_config);
            	}
>>>>>>> Resolve conflicts ocured cause Agus removed the hall_par argument from run_hall and other functions

				/* Motor Commutation loop */
				{
					HallConfig hall_config;
					qei_par qei_params;
					commutation_par commutation_params;
					int init_state;
					init_hall_config(hall_config);
					init_qei_param(qei_params);
					commutation_service(i_hall[0], i_qei[0], null, wd_interface,
					        commutation_interface, c_pwm_ctrl,
					        fet_driver_ports,
							hall_config, qei_params, commutation_params);
				}

				/* QEI Server */
				{
#ifdef DC1K
                    //connector 1 is configured as hall
                    p_ifm_encoder_hall_select_ext_d4to5 <: 0b0010;//last two bits define the interface [con2, con1], 0 - hall, 1 - QEI.
#endif
                    qei_velocity_par qei_velocity_params;
                    qei_par qei_config;
                    init_qei_velocity_params(qei_velocity_params);

                    qei_service(i_qei, encoder_ports, qei_config, qei_velocity_params);         // channel priority 1,2..6

				}

			}
		}

	}

	return 0;
}
