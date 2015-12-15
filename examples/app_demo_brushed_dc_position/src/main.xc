/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC1K-rev-c.inc>

#include <pwm_service.h>
#include <watchdog_service.h>
#include <qei_service.h>
#include <motorcontrol_service.h>

#include <position_ctrl_service.h>
#include <profile_control.h>

#include <qei_config.h>
#include <hall_config.h>
#include <motorcontrol_config.h>
#include <control_config.h>

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control)
{
	int target_position = 2000;			// ticks
	int velocity 		= 100;			// rpm
	int acceleration 	= 100;			// rpm/s
	int deceleration 	= 100;     		// rpm/s
    ProfilePositionConfig profile_position_config;

    profile_position_config.max_acceleration = MAX_ACCELERATION;
    profile_position_config.software_position_limit_max = MAX_POSITION_LIMIT;
    profile_position_config.software_position_limit_min = MIN_POSITION_LIMIT;
    profile_position_config.velocity_config.max_profile_velocity = MAX_PROFILE_VELOCITY;

    /* Initialise the position profile generator */
    init_position_profiler(profile_position_config, i_position_control);

    /* Set new target position for profile position control */
    set_profile_position(target_position, velocity, acceleration, deceleration, i_position_control);
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

int main(void)
{
    // Motor control channels
    chan c_pwm_ctrl;            // pwm channel

    interface WatchdogInterface i_watchdog[3];
    interface QEIInterface i_qei[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    interface PositionControlInterface i_position_control[3];

	par
	{

		/* Test Profile Position Client function*/
		on tile[APP_TILE]: position_profile_test(i_position_control[0]);        // test PPM on slave side

        /* XScope monitoring */
        on tile[APP_TILE]: {

            int actual_position, target_position, follow_error;

            while(1)
            {
                /* Read actual position from the Position Control Server */
                actual_position = i_position_control[1].get_position();
                target_position = i_position_control[1].get_set_position();
                follow_error = target_position - actual_position;

                xscope_int(ACTUAL_POSITION, actual_position);
                xscope_int(TARGET_POSITION, target_position);
                xscope_int(FOLLOW_ERROR, follow_error);

                delay_milliseconds(1); /* 1 ms wait */
            }
        }


		on tile[APP_TILE]:
		{
			/* Position Control Loop */
            {
                 ControlConfig position_ctrl_config;
                 init_position_control_config(position_ctrl_config); // Initialize PID parameters for Position Control

                 /* Control Loop */
                 position_control_service(position_ctrl_config, null, i_qei[1], i_motorcontrol[0],
                                             i_position_control);
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
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* QEI Service */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(qei_ports, qei_config, i_qei);
                }

                {
                    MotorcontrolConfig motorcontrol_config;
                    init_motorcontrol_config(motorcontrol_config);

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, null, i_qei[0], i_watchdog[0], i_motorcontrol);
                }

            }
		}

	}

	return 0;
}
