/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.bsp>
#include <IFM_DC100-rev-b.bsp>

#include <pwm_service.h>
#include <watchdog_service.h>
#include <qei_service.h>
#include <motorcontrol_service.h>

#include <position_ctrl_service.h>
#include <profile_control.h>

#include <user_config.h>

/* Test Profile Position function */
void position_profile_test(interface PositionControlInterface client i_position_control)
{
    int target_position = 2000;         // ticks
    int velocity        = 100;          // rpm
    int acceleration    = 100;          // rpm/s
    int deceleration    = 100;          // rpm/s

    ProfilerConfig profiler_config;
    profiler_config.polarity = POLARITY;
    profiler_config.max_position = MAX_POSITION_LIMIT;
    profiler_config.min_position = MIN_POSITION_LIMIT;

    profiler_config.max_velocity = MAX_VELOCITY;
    profiler_config.max_acceleration = MAX_ACCELERATION;
    profiler_config.max_deceleration = MAX_DECELERATION;

    /* Initialise the position profile generator */
    init_position_profiler(profiler_config, i_position_control);

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

    interface WatchdogInterface i_watchdog[2];
    interface QEIInterface i_qei[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    interface PositionControlInterface i_position_control[3];

    par
    {

        /* Test Profile Position Client function*/
        on tile[APP_TILE]: position_profile_test(i_position_control[0]);        // test PPM on slave side

        /* XScope monitoring */
        on tile[APP_TILE]: {

            int actual_position, target_position;

            while(1)
            {
                /* Read actual position from the Position Control Server */
                actual_position = i_position_control[1].get_position();
                target_position = i_position_control[1].get_target_position();

                xscope_int(ACTUAL_POSITION, actual_position/10); //Scaled for better plotting
                xscope_int(TARGET_POSITION, target_position/10); //Scaled for better plotting

                delay_milliseconds(1); /* 1 ms wait */
            }
        }

        on tile[APP_TILE]:
        {
            /* Position Control Loop */
            {
                ControlConfig position_control_config;
                position_control_config.feedback_sensor = MOTOR_FEEDBACK_SENSOR;

                position_control_config.Kp_n = POSITION_Kp;    // Divided by 10000
                position_control_config.Ki_n = POSITION_Ki;    // Divided by 10000
                position_control_config.Kd_n = POSITION_Kd;    // Divided by 10000

                position_control_config.control_loop_period = COMMUTATION_LOOP_PERIOD; //us

                /* Control Loop */
                position_control_service(position_control_config, null, i_qei[1], null, i_motorcontrol[0],
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

                /* Quadrature encoder sensor Service */
                {
                    QEIConfig qei_config;
                    qei_config.signal_type = QEI_SENSOR_SIGNAL_TYPE;           // Encoder signal type (just if applicable)
                    qei_config.index_type = QEI_SENSOR_INDEX_TYPE;             // Indexed encoder?
                    qei_config.ticks_resolution = QEI_SENSOR_RESOLUTION;       // Encoder resolution
                    qei_config.sensor_polarity = QEI_SENSOR_POLARITY;          // CW

                    qei_service(qei_ports, qei_config, i_qei);
                }

                 /* Motor Drive Service */
                 {
                    MotorcontrolConfig motorcontrol_config;
                    motorcontrol_config.motor_type = BDC_MOTOR;
                    motorcontrol_config.commutation_loop_period =  COMMUTATION_LOOP_PERIOD;

                    motorcontrol_service(fet_driver_ports, motorcontrol_config, c_pwm_ctrl, null, null, null, i_watchdog[0],
                                                 i_motorcontrol);
                 }

            }
        }

    }

    return 0;
}
