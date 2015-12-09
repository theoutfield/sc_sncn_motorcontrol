/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @brief Test illustrates usage of module_watchdog
 * @date 9/12/2015
 */
#include <pwm_service.h>
#include <hall_service.h>
#include <hall_config.h>
#include <motorcontrol_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;
HallPorts hall_ports = SOMANET_IFM_HALL_PORTS;

int main(void) {

    // Motor control interfaces
    chan c_pwm_ctrl; // pwm channels

    interface WatchdogInterface i_watchdog[3];
    interface HallInterface i_hall[5];
    interface MotorcontrolInterface i_motorcontrol[5];

    par
    {

        on tile[APP_TILE]:
        {

            i_motorcontrol[0].setVoltage(500);  // Motor start spinning

            delay_seconds(3);                   // Motor spins for 3 secs

            i_watchdog[1].disable_motors();     // Stop pinging Watchdog, motor stops

            delay_seconds(2);                   // Motor still for 2 secs

            i_motorcontrol[0].setVoltage(0);    // We deactivate spin before restarting

            i_watchdog[1].enable_motors();      // Reenable watchdog operation
            delay_milliseconds(1);
            i_watchdog[1].start();

            i_motorcontrol[0].setVoltage(500);  // Motor spins again
        }

        on tile[IFM_TILE]:
        {
            par
            {
                /* Watchdog Server */
                watchdog_service(wd_ports, i_watchdog);

                /* PWM Loop */
                pwm_service(pwm_ports, c_pwm_ctrl);

                /* Hall Server */
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
                                            c_pwm_ctrl, i_hall[0], null, i_watchdog[0], i_motorcontrol);
                }
            }
        }
    }

    return 0;
}
