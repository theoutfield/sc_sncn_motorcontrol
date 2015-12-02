/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_velocity-ctrl.xc
 * @brief Test illustrates usage of profile velocity control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <commutation_service.h>

//Position control + profile libs
#include <velocity_ctrl_service.h>
#include <profile_control.h>

//Configuration
#include <bldc_motor_config.h>
#include <qei_config.h>
#include <hall_config.h>
#include <commutation_config.h>
#include <control_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

/* Test Profile Velocity function */
void profile_velocity_test(interface VelocityControlInterface client i_velocity_control)
{
	int target_velocity = 300;	 		// rpm
	int acceleration 	= 1000;			// rpm/s
	int deceleration 	= 1000;			// rpm/s
	int actual_velocity;
	xscope_int(TARGET_VELOCITY, target_velocity);

	/* Initialise the velocity profile generator */
	init_velocity_profiler(MAX_PROFILE_VELOCITY, MAX_ACCELERATION, MAX_ACCELERATION, i_velocity_control);

	/* Set new target velocity for profile velocity control */
	set_profile_velocity( target_velocity, acceleration, deceleration, i_velocity_control);

	while(1) {
	    actual_velocity = i_velocity_control.get_velocity();

	    xscope_int(TARGET_VELOCITY, target_velocity);
	    xscope_int(ACTUAL_VELOCITY, actual_velocity);

	    delay_microseconds(1);
	}
}

int main(void)
{
	chan c_pwm_ctrl;     // pwm channel

	interface WatchdogInterface i_watchdog;
    interface CommutationInterface i_commutation[5];
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];

    interface VelocityControlInterface i_velocity_control;

	par
	{

		/* Test Profile Velocity function */
		on tile[APP_TILE]: profile_velocity_test(i_velocity_control);            // test PVM on node

		on tile[APP_TILE]:
		{

            /* Velocity Control Loop */
            {
                ControlConfig velocity_ctrl_params;
                /* Initialize PID parameters for Velocity Control (defined in config/motor/bldc_motor_config.h) */
                init_velocity_control_config(velocity_ctrl_params);

                /* Control Loop */
                velocity_control_service(velocity_ctrl_params, i_hall[1], i_qei[1],
                                            i_velocity_control, i_commutation[0]);
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
			    pwm_service(c_pwm_ctrl, pwm_ports);

                /* Watchdog Server */
			    watchdog_service(i_watchdog, wd_ports);

                /* Hall Server */
                {
                    HallConfig hall_config;
                    init_hall_config(hall_config);

                    hall_service(i_hall, hall_ports, hall_config);
                }

                /* QEI Server */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(i_qei, qei_ports, qei_config);
                }

				/* Motor Commutation loop */
                {
                    CommutationConfig commutation_config;
                    init_commutation_config(commutation_config);

                    commutation_service(i_hall[0], i_qei[0], i_watchdog, i_commutation,
                                            c_pwm_ctrl, fet_driver_ports, commutation_config);
                }
			}
		}

	}

	return 0;
}
