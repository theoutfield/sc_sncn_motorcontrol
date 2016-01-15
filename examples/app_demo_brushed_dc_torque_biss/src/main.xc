/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

/**
 * @file test_torque-ctrl.xc
 * @brief Test illustrates usage of profile torque control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <biss_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <watchdog_service.h>
#include <adc_service.h>
#include <motorcontrol_service.h>

//Torque control + profile libs
#include <torque_ctrl_service.h>
#include <profile_control.h>

//Configure your motor parameters in config/bldc_motor_config.h
#include <user_config.h>

/* Test Profile Torque Function */
void profile_torque_test(interface TorqueControlInterface client i_torque_control)
{
    int target_torque = 100;    //(desired torque/torque_constant)  * IFM resolution
    int torque_slope  = 1000;   //(desired torque_slope/torque_constant)  * IFM resolution

    ProfilerConfig profiler_config;
    profiler_config.polarity = POLARITY;
    profiler_config.max_current = MAX_CURRENT;

    /* Initialise the torque profile generator */
    init_torque_profiler(profiler_config, i_torque_control);

    /* Set new target torque for profile torque control */
    set_profile_torque(target_torque, torque_slope, i_torque_control);

    delay_seconds(3);
    target_torque = -100;

    /* Set new target torque for profile torque control */
    set_profile_torque( target_torque, torque_slope, i_torque_control);
}

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
ADCPorts adc_ports = SOMANET_IFM_ADC_PORTS;
BISSPorts biss_ports = {QEI_PORT, SOMANET_IFM_GPIO_D0, IFM_TILE_CLOCK_2};

int main(void)
{
    // Motor control channels
    chan c_pwm_ctrl, c_adctrig;

    interface WatchdogInterface i_watchdog[2];
    interface ADCInterface i_adc[2];
    interface BISSInterface i_biss[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    interface TorqueControlInterface i_torque_control[3];

    par
    {
        /* Test Profile Torque Function */
        on tile[APP_TILE]: profile_torque_test(i_torque_control[0]);

        on tile[APP_TILE]:
        {
            /* Torque Control Loop */
            ControlConfig torque_control_config;

            torque_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;

            torque_control_config.Kp_n = TORQUE_Kp;
            torque_control_config.Ki_n = TORQUE_Ki;
            torque_control_config.Kd_n = TORQUE_Kd;

            torque_control_config.control_loop_period = CONTROL_LOOP_PERIOD; // us

            /* Control Loop */
            torque_control_service(torque_control_config, i_adc[0], null, null, i_biss[1], i_motorcontrol[0], i_torque_control);
        }

        /* Currents monitoring in XScope */
        on tile[APP_TILE]:
        {
            int phaseB, phaseC, actual_torque, target_torque;

            while(1){
                {phaseB, phaseC} = i_adc[1].get_currents();
                actual_torque = i_torque_control[1].get_torque();
                target_torque = i_torque_control[1].get_target_torque();

                xscope_int(TARGET_TORQUE, target_torque);
                xscope_int(ACTUAL_TORQUE, actual_torque);
                xscope_int(PHASE_B, phaseB);
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
                /* Triggered PWM Service */
                pwm_triggered_service(pwm_ports, c_adctrig, c_pwm_ctrl);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

                /* ADC Service */
                adc_service(adc_ports, c_adctrig, i_adc);

                /* BiSS service */
                {
                    BISSConfig biss_config;
                    biss_config.multiturn_length = BISS_MULTITURN_LENGTH;
                    biss_config.multiturn_resolution = BISS_MULTITURN_RESOLUTION;
                    biss_config.singleturn_length = BISS_SINGLETURN_LENGTH;
                    biss_config.singleturn_resolution = BISS_SINGLETURN_RESOLUTION;
                    biss_config.status_length = BISS_STATUS_LENGTH;
                    biss_config.crc_poly = BISS_CRC_POLY;
                    biss_config.pole_pairs = POLE_PAIRS;
                    biss_config.polarity = BISS_POLARITY;
                    biss_config.clock_dividend = BISS_CLOCK_DIVIDEND;
                    biss_config.clock_divisor = BISS_CLOCK_DIVISOR;
                    biss_config.timeout = BISS_TIMEOUT;
                    biss_config.max_ticks = BISS_MAX_TICKS;
                    biss_config.velocity_loop = BISS_VELOCITY_LOOP;
                    biss_config.offset_electrical = BISS_OFFSET_ELECTRICAL;

                    biss_service(biss_ports, biss_config, i_biss);
                }

                /* Motor Commutation loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BDC_MOTOR;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                            c_pwm_ctrl, null, null, null, i_watchdog[0], i_motorcontrol);
                }
            }
        }

    }

    return 0;
}
