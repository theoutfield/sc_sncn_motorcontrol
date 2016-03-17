/* INCLUDE BOARD SUPPORT FILES FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_velocity-ctrl.xc
 * @brief Test illustrates usage of profile velocity control
 * @author Synapticon GmbH (www.synapticon.com)
 */

//BLDC Motor drive libs
#include <qei_service.h>
#include <hall_service.h>
#include <pwm_service.h>
#include <watchdog_service.h>
#include <motorcontrol_service.h>

//Position control + profiler libs
#include <velocity_ctrl_service.h>
#include <profile_control.h>

//Configuration
#include <user_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;
#if(MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
BISSPorts biss_ports = SOMANET_IFM_BISS_PORTS;
#elif(MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
QEIPorts qei_ports = SOMANET_IFM_QEI_PORTS;
#endif

/* Test Profile Velocity function */
void profile_velocity_test(interface VelocityControlInterface client i_velocity_control)
{
    int target_velocity = 900;          // rpm
    int acceleration    = 100;          // rpm/s
    int deceleration    = 100;          // rpm/s

    ProfilerConfig profiler_config;
    profiler_config.max_velocity = MAX_VELOCITY;
    profiler_config.max_acceleration = MAX_ACCELERATION;
    profiler_config.max_deceleration = MAX_DECELERATION;

    /* Initialise the velocity profile generator */
    init_velocity_profiler(profiler_config, i_velocity_control);

    /* Set new target velocity for profile velocity control */
    set_profile_velocity(target_velocity, acceleration, deceleration, i_velocity_control);
}

int main(void)
{
    chan c_pwm_ctrl;     // pwm channel

    interface WatchdogInterface i_watchdog[2];
    interface HallInterface i_hall[5];
    interface MotorcontrolInterface i_motorcontrol[4];
    interface VelocityControlInterface i_velocity_control[3];
#if(MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
    interface BISSInterface i_biss[5];
#elif(MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
    interface QEIInterface i_qei[5];
#endif

    par
    {

        /* Test Profile Velocity function */
        on tile[APP_TILE]: profile_velocity_test(i_velocity_control[0]);

        on tile[APP_TILE]:
        /* XScope monitoring */
        {
            int target_velocity, actual_velocity;

            while(1) {

                actual_velocity = i_velocity_control[1].get_velocity();
                target_velocity = i_velocity_control[1].get_target_velocity();

                xscope_int(TARGET_VELOCITY, target_velocity);
                xscope_int(ACTUAL_VELOCITY, actual_velocity);

                delay_milliseconds(1);
            }
        }

        on tile[APP_TILE]:
        /* Velocity Control Service */
        {
            ControlConfig velocity_control_config;

            velocity_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;

            velocity_control_config.Kp_n = VELOCITY_Kp;
            velocity_control_config.Ki_n = VELOCITY_Ki;
            velocity_control_config.Kd_n = VELOCITY_Kd;

            velocity_control_config.control_loop_period =  CONTROL_LOOP_PERIOD;

            /* Control Loop */
#if(MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
            velocity_control_service(velocity_control_config, i_hall[1], null, i_biss[1], i_motorcontrol[0],
                                        i_velocity_control);
#elif(MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
            velocity_control_service(velocity_control_config, i_hall[1], i_qei[1], null, i_motorcontrol[0],
                                        i_velocity_control);
#else
            velocity_control_service(velocity_control_config, i_hall[1], null, null, i_motorcontrol[0],
                                        i_velocity_control);
#endif
        }

        /************************************************************
         * IFM_CORE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            par
            {
                /* PWM Service */
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Watchdog Service */
                watchdog_service(wd_ports, i_watchdog);

                /* Hall sensor Service */
                {
                    HallConfig hall_config;
                    hall_config.pole_pairs = POLE_PAIRS;

                    hall_service(hall_ports, hall_config, i_hall);
                }

#if(MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
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
#elif(MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
                /* Quadrature encoder sensor Service */
                {
                    QEIConfig qei_config;
                    qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;       // Encoder signal type (just if applicable)
                    qei_config.index_type = QEI_SENSOR_INDEX_TYPE;         // Indexed encoder?
                    qei_config.ticks_resolution = QEI_SENSOR_RESOLUTION;   // Encoder resolution
                    qei_config.sensor_polarity = QEI_SENSOR_POLARITY;      // CW

                    qei_service(qei_ports, qei_config, i_qei);
                }
#endif

                /* Motor Commutation Service */
                {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BLDC_MOTOR;
                    motorcontrol_config.polarity_type = POLARITY;
                    motorcontrol_config.commutation_sensor = MOTOR_COMMUTATION_SENSOR;
                    motorcontrol_config.bldc_winding_type = BLDC_WINDING_TYPE;
                    motorcontrol_config.hall_offset[0] =  COMMUTATION_OFFSET_CLK;
                    motorcontrol_config.hall_offset[1] = COMMUTATION_OFFSET_CCLK;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

#if(MOTOR_FEEDBACK_SENSOR == BISS_SENSOR)
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_hall[0], null, i_biss[0], i_watchdog[0], i_motorcontrol);
#elif(MOTOR_FEEDBACK_SENSOR == QEI_SENSOR)
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_hall[0], i_qei[0], null, i_watchdog[0], i_motorcontrol);
#else
                    motorcontrol_service(fet_driver_ports, motorcontrol_config,
                                         c_pwm_ctrl, i_hall[0], null, null, i_watchdog[0], i_motorcontrol);
#endif
                }
            }
        }
    }

    return 0;
}
