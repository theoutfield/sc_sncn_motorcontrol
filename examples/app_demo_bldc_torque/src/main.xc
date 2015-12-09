/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @file test_torque-ctrl.xc
 * @brief Test illustrates usage of profile torque control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <adc_service.h>
#include <motorcontrol_service.h>

//Torque control + profile libs
#include <torque_ctrl_service.h>
#include <profile_control.h>

//Configure your motor parameters in config/bldc_motor_config.h
#include <qei_config.h>
#include <hall_config.h>
#include <motorcontrol_config.h>
#include <control_config.h>

/* Test Profile Torque Function */
void profile_torque_test(interface TorqueControlInterface client i_torque_control)
{
	int target_torque = 400; 	//(desired torque/torque_constant)  * IFM resolution
	int torque_slope  = 1000;  	//(desired torque_slope/torque_constant)  * IFM resolution

	ProfileTorqueConfig profile_torque_config;
	profile_torque_config.max_torque = MOTOR_TORQUE_CONSTANT * MAX_NOMINAL_CURRENT * IFM_RESOLUTION;
	profile_torque_config.polarity = POLARITY;

	/* Initialise the torque profile generator */
	init_torque_profiler(profile_torque_config, i_torque_control);

	/* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, i_torque_control);

	delay_seconds(3);
	target_torque = -400;

    /* Set new target torque for profile torque control */
	set_profile_torque( target_torque, torque_slope, i_torque_control);
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;

int main(void)
{
	// Motor control channels
	chan c_adctrig, c_pwm_ctrl;

	interface WatchdogInterface i_watchdog[3];
    interface ADCInterface i_adc[3];
    interface HallInterface i_hall[5];
    interface QEIInterface i_qei[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    interface TorqueControlInterface i_torque_control[3];

	par
	{
		/* Test Profile Torque Function */
		on tile[APP_TILE]: profile_torque_test(i_torque_control[0]);

		on tile[APP_TILE]:
		{
		    /* Torque Control Loop */
            ControlConfig torque_ctrl_params;
            init_torque_control_config(torque_ctrl_params);  // Initialize PID parameters for Torque Control

            /* Control Loop */
            torque_control_service(torque_ctrl_params, i_adc[0], i_motorcontrol[0],  i_hall[1], i_qei[1], i_torque_control);
        }

		/* Currents monitoring in XScope */
		on tile[APP_TILE]:
		{
		    int phaseB, phaseC, actual_torque, target_torque;
		    unsigned hall_state = 0;
		    while(1){
		        {phaseB, phaseC} = i_adc[1].get_currents();
		        actual_torque = i_torque_control[1].get_torque();
		        target_torque = i_torque_control[1].get_set_torque();

		        xscope_int(TARGET_TORQUE, target_torque);
		        xscope_int(ACTUAL_TORQUE, actual_torque);
		        xscope_int(PHASE_B, phaseB);
		        xscope_int(PHASE_C, phaseC);
		        delay_microseconds(50);
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
			    pwm_triggered_service(pwm_ports, c_pwm_ctrl, c_adctrig);

                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* ADC Loop */
                adc_service(i_adc, adc_ports, c_adctrig);

                /* QEI Service */
                {
                    QEIConfig qei_config;
                    init_qei_config(qei_config);

                    qei_service(qei_ports, qei_config, i_qei);
                }

                {
                    HallConfig hall_config;
                    init_hall_config(hall_config);

                    hall_service(hall_ports, hall_config, i_hall);
                }

                /* Motor Commutation loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    init_motorcontrol_config(motorcontrol_config);

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, i_hall[0], i_qei[0], i_watchdog[0], i_motorcontrol);
                }
			}
		}

	}

	return 0;
}
