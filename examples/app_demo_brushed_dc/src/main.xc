/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IFM_BOARD_REQUIRED" WITH AN APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_C22-rev-a.inc>
#include <IFM_DC100-rev-b.inc>

/**
 * @brief Test illustrates usage of module_commutation
 * @date 17/06/2014
 */
#include <pwm_service.h>
#include <watchdog_service.h>
#include <motorcontrol_service.h>

#include <motorcontrol_config.h>

PwmPorts pwm_ports = SOMANET_IFM_PWM_PORTS;
WatchdogPorts wd_ports = SOMANET_IFM_WATCHDOG_PORTS;
FetDriverPorts fet_driver_ports = SOMANET_IFM_FET_DRIVER_PORTS;

int main(void) {

    // Motor control channels
    chan c_pwm_ctrl;

    interface WatchdogInterface i_watchdog;
    interface MotorcontrolInterface i_motorcontrol[5];

    par
    {
        /************************************************************
         * USER_TILE
         ************************************************************/

        on tile[APP_TILE_1]:
        {
              while (1) {
                  i_motorcontrol[0].setVoltage(1000);
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

                /* Brushed Motor Drive loop */
                {
                    MotorcontrolConfig motorcontrol_config;
                    init_motorcontrol_config(motorcontrol_config);

                    motorcontrol_service(fet_driver_ports, motorcontrol_config, c_pwm_ctrl, null, null, i_watchdog, i_motorcontrol);
                }
            }
        }

    }

    return 0;
}
