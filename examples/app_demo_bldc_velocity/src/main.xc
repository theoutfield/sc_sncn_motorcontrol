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
#include <motorcontrol_service.h>

//Position control + profile libs
#include <velocity_ctrl_service.h>
#include <profile_control.h>

//Configuration
#include <bldc_motor_config.h>
#include <motorcontrol_config.h>
#include <control_config.h>
#include <stdlib.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

/* Test Profile Velocity function */
void profile_velocity_test(interface VelocityControlInterface client i_velocity_control)
{
	int target_velocity = 1000;	 		// rpm
	int acceleration 	= 50;			// rpm/s
	int deceleration 	= 50;			// rpm/s
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
    interface MotorcontrolInterface i_motorcontrol[5];
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
                ControlConfig velocity_ctrl_config;
                /* Initialize PID parameters for Velocity Control (defined in config/motor/bldc_motor_config.h) */
                init_velocity_control_config(velocity_ctrl_config);

                /* Control Loop */
                velocity_control_service(velocity_ctrl_config, i_hall[1], i_qei[1], i_velocity_control, i_motorcontrol[0]);

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
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* QEI Service */
                {
                    QEIConfig qei_config;
                    qei_config.index = QEI_SENSOR_TYPE;
                    qei_config.real_counts = ENCODER_RESOLUTION;
                    qei_config.sensor_polarity = QEI_SENSOR_POLARITY;
                    qei_config.poles = POLE_PAIRS;
                    qei_service(qei_ports, qei_config, i_qei);
                }

                /* Hall Service*/
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;
                    hall_service(hall_ports, hall_config, i_hall);
                }

                /* Motor Commutation loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.angle_variance = (60 * 4096) / (POLE_PAIRS * 2 * 360);

                    if (POLE_PAIRS < 4) {
                        motorcontrol_config.nominal_speed =  MAX_NOMINAL_SPEED * 4;
                    } else if (POLE_PAIRS >= 4) {
                        motorcontrol_config.nominal_speed =  MAX_NOMINAL_SPEED;
                    }

                    motorcontrol_config.commutation_loop_freq =  COMMUTATION_LOOP_FREQUENCY_KHZ;
                    motorcontrol_config.hall_offset_clk =  COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset_cclk = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.bldc_winding_type = WINDING_TYPE;
                    motorcontrol_config.qei_forward_offset = 0;
                    motorcontrol_config.qei_backward_offset = 0;
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], i_qei[0], i_watchdog, i_motorcontrol);
                }

            }
		}

	}

	return 0;
}
